import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { DebuggingDashboard } from '../DebuggingDashboard';

vi.mock('../../../hooks/useROS', () => ({
  useROS: () => ({
    isConnected: false,
    ros: null
  })
}));

describe('DebuggingDashboard', () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it('renders with critical state strip visible', () => {
    render(<DebuggingDashboard />);
    expect(screen.getByRole('status')).toBeInTheDocument();
    expect(screen.getByText('Disconnected')).toBeInTheDocument();
  });

  it('renders four tabs: Sensor Health, Blackboard, Behavior Tree, System Analytics', () => {
    render(<DebuggingDashboard />);
    expect(screen.getByRole('button', { name: /sensor health tab/i })).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /blackboard tab/i })).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /behavior tree tab/i })).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /system analytics tab/i })).toBeInTheDocument();
  });

  it('shows Sensor Health content by default', () => {
    render(<DebuggingDashboard />);
    expect(screen.getByText('Sensor Health Dashboard')).toBeInTheDocument();
  });

  it('switches to Blackboard tab when clicked', async () => {
    const user = userEvent.setup();
    render(<DebuggingDashboard />);
    await user.click(screen.getByRole('button', { name: /blackboard tab/i }));
    expect(screen.getByPlaceholderText(/search keys/i)).toBeInTheDocument();
  });

  it('switches to Behavior Tree tab when clicked', async () => {
    const user = userEvent.setup();
    render(<DebuggingDashboard />);
    await user.click(screen.getByRole('button', { name: /behavior tree tab/i }));
    expect(screen.getByText('Execution tree')).toBeInTheDocument();
  });

  it('switches to System Analytics tab when clicked', async () => {
    const user = userEvent.setup();
    render(<DebuggingDashboard />);
    await user.click(screen.getByRole('button', { name: /system analytics tab/i }));
    expect(screen.getByText('Incident replay')).toBeInTheDocument();
    expect(screen.getByText('Resource usage')).toBeInTheDocument();
  });

  it('has application role and accessible name', () => {
    render(<DebuggingDashboard />);
    const app = screen.getByRole('application', { name: /debugging dashboard/i });
    expect(app).toBeInTheDocument();
  });

  it('marks active tab with aria-pressed', async () => {
    const user = userEvent.setup();
    render(<DebuggingDashboard />);
    const sensorTab = screen.getByRole('button', { name: /sensor health tab/i });
    expect(sensorTab).toHaveAttribute('aria-pressed', 'true');
    await user.click(screen.getByRole('button', { name: /blackboard tab/i }));
    expect(screen.getByRole('button', { name: /blackboard tab/i })).toHaveAttribute('aria-pressed', 'true');
  });
});
