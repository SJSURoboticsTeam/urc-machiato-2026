import React, { useState, useCallback } from 'react';
import { Send, Radio } from 'lucide-react';
import { UI_CONSTANTS } from '../../constants/uiConstants';
import { generateMessageDelay } from '../../utils/mockDataUtils';
import ROSLIB from '../../utils/rosbridge';

/**
 * Message Tester Component
 * Handles sending test messages through different communication channels
 */
export const MessageTester = ({
  messageHistory,
  setMessageHistory,
  setFlashingChannel,
  setLastMessageFlow,
  commMetrics,
  setCommMetrics
}) => {
  const [testMessage, setTestMessage] = useState('');
  const [messageTarget, setMessageTarget] = useState(UI_CONSTANTS.MESSAGE_TARGETS[0]);
  const [canMessageType, setCanMessageType] = useState(UI_CONSTANTS.CAN_MESSAGE_TYPES[0]);
  const [isSending, setIsSending] = useState(false);
  const [sendError, setSendError] = useState(null);

  const sendTestMessage = useCallback(async () => {
    if (!testMessage.trim() && messageTarget !== 'can') return;
    if (isSending) return; // Prevent double-sending

    setIsSending(true);
    setSendError(null);

    const timestamp = Date.now();

    try {
      if (messageTarget === 'can') {
        // Send CAN message to ROS2
        const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

        // Wait for connection
        await new Promise((resolve, reject) => {
          ros.on('connection', () => resolve());
          ros.on('error', (error) => reject(error));
          ros.connect();

          // Timeout after 5 seconds
          setTimeout(() => reject(new Error('Connection timeout')), 5000);
        });

        const canTopic = new ROSLIB.Topic({
          ros: ros,
          name: '/can/data_request',
          messageType: 'std_msgs/String'
        });

        const canMessage = {
          type: canMessageType,
          data: testMessage || 'test_data',
          timestamp: timestamp,
          source: 'dashboard_test'
        };

        canTopic.publish(new ROSLIB.Message({
          data: JSON.stringify(canMessage)
        }));

        ros.close();
      }

      const message = {
        id: `msg_${timestamp}`,
        content: messageTarget === 'can' ? `${canMessageType}: ${testMessage}` : testMessage,
        target: messageTarget,
        timestamp,
        status: 'sent'
      };

      // Add to history
      setMessageHistory(prev => [message, ...prev.slice(0, UI_CONSTANTS.MESSAGE_HISTORY_LIMIT)]);

      // Visual feedback
      setFlashingChannel(messageTarget);
      setTimeout(() => setFlashingChannel(null), UI_CONSTANTS.FLASH_DURATION);

      // Simulate message flow
      setLastMessageFlow({ from: 'user', to: messageTarget, timestamp });

      // Update metrics
      setCommMetrics(prev => ({
      ...prev,
      [messageTarget]: {
        ...prev[messageTarget],
        sent: prev[messageTarget].sent + 1,
        lastMsg: messageTarget === 'can' ? canMessageType : testMessage,
        active: true
      }
    }));

    // Clear input
    setTestMessage('');

    // Simulate response after delay
    setTimeout(() => {
      setMessageHistory(prev =>
        prev.map(msg =>
          msg.id === message.id
            ? { ...msg, status: 'delivered', response: 'ACK' }
            : msg
        )
      );

      setCommMetrics(prev => ({
        ...prev,
        [messageTarget]: {
          ...prev[messageTarget],
          received: prev[messageTarget].received + 1
        }
      }));
    }, generateMessageDelay());

    } catch (error) {
      console.error('Error sending message:', error);
      setSendError(`Failed to send message: ${error.message}`);

      // Add error message to history
      const errorMessage = {
        id: `msg_${timestamp}_error`,
        content: `Error: ${error.message}`,
        target: messageTarget,
        timestamp,
        status: 'error'
      };
      setMessageHistory(prev => [errorMessage, ...prev.slice(0, UI_CONSTANTS.MESSAGE_HISTORY_LIMIT)]);
    } finally {
      setIsSending(false);
    }
  }, [testMessage, messageTarget, canMessageType, isSending, setMessageHistory, setFlashingChannel, setLastMessageFlow, setCommMetrics]);

  const getChannelColor = (channel) => {
    switch (channel) {
      case 'websocket': return 'text-blue-400';
      case 'ros2': return 'text-green-400';
      case 'can': return 'text-purple-400';
      default: return 'text-gray-400';
    }
  };

  return (
    <div className="space-y-4">
      <h3 className="text-lg font-semibold text-zinc-200 flex items-center gap-2">
        <Send className="w-5 h-5" />
        Message Tester
      </h3>

      {/* Message input */}
      <div className="space-y-3">
        <div className="flex gap-2">
          <input
            type="text"
            value={testMessage}
            onChange={(e) => setTestMessage(e.target.value)}
            onKeyDown={(e) => e.key === 'Enter' && sendTestMessage()}
            placeholder="Enter test message..."
            className="flex-1 px-3 py-2 bg-zinc-800 border border-zinc-600 rounded text-zinc-200 placeholder-zinc-500 focus:border-blue-500 focus:outline-none"
          />
          <button
            onClick={sendTestMessage}
            disabled={!testMessage.trim() || isSending}
            className="px-4 py-2 bg-blue-600 hover:bg-blue-700 disabled:bg-zinc-600 disabled:cursor-not-allowed text-white rounded flex items-center gap-2 transition-colors"
          >
            {isSending ? (
              <div className="animate-spin rounded-full h-4 w-4 border-b-2 border-white"></div>
            ) : (
              <Send className="w-4 h-4" />
            )}
            {isSending ? 'Sending...' : 'Send'}
          </button>
        </div>

        {/* Loading and Error States */}
        {isSending && (
          <div className="flex items-center gap-2 text-blue-400 text-sm">
            <div className="animate-spin rounded-full h-4 w-4 border-b-2 border-blue-400"></div>
            Sending message...
          </div>
        )}

        {sendError && (
          <div className="bg-red-900/20 border border-red-500/50 rounded p-3">
            <p className="text-red-400 text-sm">{sendError}</p>
          </div>
        )}

        {/* Channel selector */}
        <div className="flex gap-2">
          {UI_CONSTANTS.MESSAGE_TARGETS.map(channel => (
            <button
              key={channel}
              onClick={() => setMessageTarget(channel)}
              className={`px-3 py-1 rounded text-sm font-medium transition-colors ${
                messageTarget === channel
                  ? 'bg-blue-600 text-white'
                  : 'bg-zinc-700 text-zinc-300 hover:bg-zinc-600'
              }`}
            >
              <Radio className="w-3 h-3 inline mr-1" />
              {channel.toUpperCase()}
            </button>
          ))}
        </div>

        {/* CAN Message Type Selector */}
        {messageTarget === 'can' && (
          <div className="space-y-2">
            <label className="text-sm text-zinc-400">CAN Message Type:</label>
            <select
              value={canMessageType}
              onChange={(e) => setCanMessageType(e.target.value)}
              className="w-full px-3 py-2 bg-zinc-800 border border-zinc-600 rounded text-zinc-200 focus:border-purple-500 focus:outline-none"
            >
              {UI_CONSTANTS.CAN_MESSAGE_TYPES.map(type => (
                <option key={type} value={type}>
                  {type.replace('can_', '').replace('_', ' ').toUpperCase()}
                </option>
              ))}
            </select>
          </div>
        )}
      </div>

      {/* Message history */}
      <div className="space-y-2">
        <h4 className="text-sm font-medium text-zinc-400">Recent Messages</h4>
        <div className="max-h-48 overflow-y-auto space-y-1">
          {messageHistory.slice(0, UI_CONSTANTS.MESSAGE_HISTORY_LIMIT).map(msg => (
            <div key={msg.id || Math.random()} className="text-xs bg-zinc-800 p-2 rounded flex justify-between items-center">
              <div className="flex items-center gap-2">
                <span className={getChannelColor(msg.target || 'unknown')}>
                  {(msg.target || 'UNKNOWN').toUpperCase()}
                </span>
                <span className="text-zinc-300 truncate max-w-32">{msg.content || 'No content'}</span>
              </div>
              <span className={`text-xs px-1 rounded ${
                msg.status === 'sent' ? 'bg-yellow-900 text-yellow-300' :
                msg.status === 'delivered' ? 'bg-green-900 text-green-300' :
                msg.status === 'received' ? 'bg-blue-900 text-blue-300' :
                'bg-zinc-700 text-zinc-400'
              }`}>
                {msg.status || 'unknown'}
              </span>
            </div>
          ))}
          {messageHistory.length === 0 && (
            <div className="text-xs text-zinc-500 text-center py-4">
              No messages sent yet
            </div>
          )}
        </div>
      </div>
    </div>
  );
};
