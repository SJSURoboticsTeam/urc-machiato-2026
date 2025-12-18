import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { MessageTester } from './MessageTester';

describe('MessageTester', () => {
  const mockProps = {
    messageHistory: [],
    setMessageHistory: vi.fn(),
    setFlashingChannel: vi.fn(),
    setLastMessageFlow: vi.fn(),
    commMetrics: {
      websocket: { sent: 0, received: 0, lastMsg: null, active: false },
      ros2: { sent: 0, received: 0, lastMsg: null, active: false },
      can: { sent: 0, received: 0, lastMsg: null, active: false, isMock: true }
    },
    setCommMetrics: vi.fn()
  };

  beforeEach(() => {
    vi.clearAllMocks();
  });

  it('renders message tester interface', () => {
    render(<MessageTester {...mockProps} />);

    expect(screen.getByText('Message Tester')).toBeInTheDocument();
    expect(screen.getByPlaceholderText('Enter test message...')).toBeInTheDocument();
    expect(screen.getByText('Send')).toBeInTheDocument();
    expect(screen.getByText('WEBSOCKET')).toBeInTheDocument();
    expect(screen.getByText('ROS2')).toBeInTheDocument();
    expect(screen.getByText('CAN')).toBeInTheDocument();
  });

  it('enables send button when target is selected', () => {
    render(<MessageTester {...mockProps} />);

    const sendButton = screen.getByText('Send');
    expect(sendButton).toBeDisabled();

    // Select WEBSOCKET target
    fireEvent.click(screen.getByText('WEBSOCKET'));
    expect(sendButton).toBeDisabled(); // Still disabled without message

    // Type a message
    const input = screen.getByPlaceholderText('Enter test message...');
    fireEvent.change(input, { target: { value: 'test message' } });

    expect(sendButton).not.toBeDisabled();
  });

  it('sends message when send button is clicked', () => {
    render(<MessageTester {...mockProps} />);

    // Select target and type message
    fireEvent.click(screen.getByText('WEBSOCKET'));
    const input = screen.getByPlaceholderText('Enter test message...');
    fireEvent.change(input, { target: { value: 'test message' } });

    // Click send
    fireEvent.click(screen.getByText('Send'));

    // Check that callbacks were called
    expect(mockProps.setMessageHistory).toHaveBeenCalledWith(
      expect.arrayContaining([
        expect.objectContaining({
          content: 'test message',
          target: 'websocket',
          status: 'sent'
        })
      ])
    );

    expect(mockProps.setFlashingChannel).toHaveBeenCalledWith('websocket');
    expect(mockProps.setLastMessageFlow).toHaveBeenCalledWith(
      expect.objectContaining({
        from: 'user',
        to: 'websocket'
      })
    );

    expect(mockProps.setCommMetrics).toHaveBeenCalledWith(
      expect.objectContaining({
        websocket: expect.objectContaining({
          sent: 1,
          lastMsg: 'test message',
          active: true
        })
      })
    );
  });

  it('switches between different targets', () => {
    render(<MessageTester {...mockProps} />);

    // Default should be WEBSOCKET selected
    expect(screen.getByText('WEBSOCKET')).toHaveClass('bg-blue-600');

    // Click ROS2
    fireEvent.click(screen.getByText('ROS2'));
    expect(screen.getByText('ROS2')).toHaveClass('bg-blue-600');

    // Click CAN
    fireEvent.click(screen.getByText('CAN'));
    expect(screen.getByText('CAN')).toHaveClass('bg-blue-600');
  });

  it('displays recent messages', () => {
    const propsWithHistory = {
      ...mockProps,
      messageHistory: [
        { id: '1', content: 'test message 1', target: 'websocket', timestamp: Date.now(), status: 'sent' },
        { id: '2', content: 'test message 2', target: 'ros2', timestamp: Date.now(), status: 'sent' }
      ]
    };

    render(<MessageTester {...propsWithHistory} />);

    expect(screen.getByText('Recent Messages')).toBeInTheDocument();
    expect(screen.getByText('WEBSOCKET')).toBeInTheDocument();
    expect(screen.getByText('test message 1')).toBeInTheDocument();
    expect(screen.getByText('ROS2')).toBeInTheDocument();
    expect(screen.getByText('test message 2')).toBeInTheDocument();
  });

  it('shows empty state when no messages', () => {
    render(<MessageTester {...mockProps} />);

    expect(screen.getByText('No messages sent yet')).toBeInTheDocument();
  });

  it('clears input after sending', () => {
    render(<MessageTester {...mockProps} />);

    // Select target and type message
    fireEvent.click(screen.getByText('WEBSOCKET'));
    const input = screen.getByPlaceholderText('Enter test message...');
    fireEvent.change(input, { target: { value: 'test message' } });

    // Click send
    fireEvent.click(screen.getByText('Send'));

    // Input should be cleared
    expect(input.value).toBe('');
  });
});
