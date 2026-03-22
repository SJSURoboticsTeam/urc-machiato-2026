/**
 * Connection state for data sources.
 * Provides unified status for ROS, Mock, and Replay sources.
 */
export type ConnectionState =
  | { status: 'connected'; since: Date }
  | { status: 'disconnected' }
  | { status: 'connecting'; attempt: number }
  | { status: 'error'; error: Error; retryable: boolean };

/**
 * Abstract interface for all data sources (ROS, Mock, Replay, etc.).
 *
 * Provides unified API for:
 * - Connection management
 * - Topic subscriptions
 * - State queries
 */
export interface IDataSource {
  /**
   * Establish connection to data source.
   * @throws {Error} if connection fails
   */
  connect(): Promise<void>;

  /**
   * Close connection and cleanup resources.
   */
  disconnect(): Promise<void>;

  /**
   * Subscribe to a data stream.
   * @param topic - Topic identifier (e.g., '/robot/state', 'telemetry')
   * @param callback - Called with data when received
   * @returns Unsubscribe function
   */
  subscribe<T>(topic: string, callback: (data: T) => void): () => void;

  /**
   * Get current connection state.
   */
  getConnectionState(): ConnectionState;

  /**
   * Get data source type (for debugging/UI).
   */
  getSourceType(): 'ros' | 'mock' | 'replay';
}
