import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { MessageTester } from '../MessageTester';
import { UI_CONSTANTS } from '../../../constants/uiConstants';
import { vi, describe, test, expect, beforeEach } from 'vitest';

// Mock the useROS hook
vi.mock('../../../hooks/useROS', () => ({
  useROS: () => ({
    ros: { isConnected: true },
    isConnected: true,
    subscribe: vi.fn(),
    publish: vi.fn(),
    callService: vi.fn(),
    error: null,
  }),
}));

describe('MessageTester Component', () => {
  const mockProps = {
    messageHistory: [],
    setMessageHistory: vi.fn(),
    setFlashingChannel: vi.fn(),
    setLastMessageFlow: vi.fn(),
    commMetrics: {
      websocket: { sent: 0, received: 0, lastMsg: '', active: false },
      ros2: { sent: 0, received: 0, lastMsg: '', active: false },
      can: { sent: 0, received: 0, lastMsg: '', active: false },
    },
    setCommMetrics: vi.fn(),
  };

  beforeEach(() => {
    vi.clearAllMocks();
  });

  test('renders message tester component', () => {
    render(<MessageTester {...mockProps} />);
    expect(screen.getByText('Message Tester')).toBeInTheDocument();
    expect(screen.getByPlaceholderText('Enter test message...')).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /send/i })).toBeInTheDocument();
  });

  test('displays channel selector buttons', () => {
    render(<MessageTester {...mockProps} />);

    UI_CONSTANTS.MESSAGE_TARGETS.forEach(channel => {
      expect(screen.getByText(channel.toUpperCase())).toBeInTheDocument();
    });
  });

  test('shows CAN message type selector when CAN is selected', () => {
    render(<MessageTester {...mockProps} />);

    // Initially CAN options should not be visible
    expect(screen.queryByText('CAN Message Type:')).not.toBeInTheDocument();

    // Click CAN button
    fireEvent.click(screen.getByText('CAN'));

    // CAN message type selector should now be visible
    expect(screen.getByText('CAN Message Type:')).toBeInTheDocument();

    // Check that CAN message types are available
    UI_CONSTANTS.CAN_MESSAGE_TYPES.forEach(type => {
      const displayText = type.replace('can_', '').replace('_', ' ').toUpperCase();
      expect(screen.getByText(displayText)).toBeInTheDocument();
    });
  });

  test('updates message input value', () => {
    render(<MessageTester {...mockProps} />);
    const input = screen.getByPlaceholderText('Enter test message...');

    fireEvent.change(input, { target: { value: 'test message' } });
    expect(input.value).toBe('test message');
  });

  test('handles Enter key press to send message', () => {
    render(<MessageTester {...mockProps} />);
    const input = screen.getByPlaceholderText('Enter test message...');

    fireEvent.change(input, { target: { value: 'test message' } });
    fireEvent.keyPress(input, { key: 'Enter', code: 'Enter' });

    expect(mockProps.setMessageHistory).toHaveBeenCalled();
    expect(mockProps.setFlashingChannel).toHaveBeenCalledWith('websocket');
  });

  test('sends message when button is clicked', () => {
    render(<MessageTester {...mockProps} />);
    const input = screen.getByPlaceholderText('Enter test message...');
    const sendButton = screen.getByRole('button', { name: /send/i });

    fireEvent.change(input, { target: { value: 'test message' } });
    fireEvent.click(sendButton);

    expect(mockProps.setMessageHistory).toHaveBeenCalled();
    expect(mockProps.setFlashingChannel).toHaveBeenCalledWith('websocket');
  });

  test('disables send button when message is empty', () => {
    render(<MessageTester {...mockProps} />);
    const sendButton = screen.getByRole('button', { name: /send/i });

    expect(sendButton).toBeDisabled();

    const input = screen.getByPlaceholderText('Enter test message...');
    fireEvent.change(input, { target: { value: 'test' } });

    expect(sendButton).not.toBeDisabled();
  });

  test('updates communication metrics on message send', () => {
    render(<MessageTester {...mockProps} />);
    const input = screen.getByPlaceholderText('Enter test message...');
    const sendButton = screen.getByRole('button', { name: /send/i });

    fireEvent.change(input, { target: { value: 'test message' } });
    fireEvent.click(sendButton);

    expect(mockProps.setCommMetrics).toHaveBeenCalled();
  });

  test('handles CAN message sending', () => {
    render(<MessageTester {...mockProps} />);

    // Select CAN channel
    fireEvent.click(screen.getByText('CAN'));

    // Select CAN message type
    const select = screen.getByRole('combobox');
    fireEvent.change(select, { target: { value: 'can_sensor_request' } });

    // Send message
    const sendButton = screen.getByRole('button', { name: /send/i });
    fireEvent.click(sendButton);

    expect(mockProps.setMessageHistory).toHaveBeenCalled();
  });

  test('clears input after sending message', async () => {
    render(<MessageTester {...mockProps} />);
    const input = screen.getByPlaceholderText('Enter test message...');

    fireEvent.change(input, { target: { value: 'test message' } });
    fireEvent.click(screen.getByRole('button', { name: /send/i }));

    await waitFor(() => {
      expect(input.value).toBe('');
    });
  });

  test('respects message history limit', () => {
    const longHistory = Array(UI_CONSTANTS.MESSAGE_HISTORY_LIMIT + 5)
      .fill(null)
      .map((_, i) => ({ id: `msg_${i}`, content: `message ${i}` }));

    render(<MessageTester {...mockProps} messageHistory={longHistory} />);

    expect(mockProps.setMessageHistory).toHaveBeenCalledWith(
      expect.arrayContaining(longHistory.slice(-UI_CONSTANTS.MESSAGE_HISTORY_LIMIT))
    );
  });

  test('applies correct channel color styling', () => {
    render(<MessageTester {...mockProps} />);

    // Check that channel buttons have correct styling
    const websocketButton = screen.getByText('WEBSOCKET');
    expect(websocketButton).toHaveClass('bg-blue-600'); // Should be selected by default
  });
});
