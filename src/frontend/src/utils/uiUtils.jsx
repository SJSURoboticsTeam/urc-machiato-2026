/**
 * UI Utility Functions - Extracted logic for reusability
 * Simplifies complex conditionals and status handling
 */

import { STATUS_STYLES, PRIORITY_STYLES } from '../constants/uiConstants';

/**
 * Get CSS classes for a status
 * @param {string} status - Status string
 * @returns {string} CSS classes
 */
export const getStatusClasses = (status) =>
  STATUS_STYLES[status] || STATUS_STYLES.unknown;

/**
 * Get CSS classes for a priority badge
 * @param {number} priority - Priority level (1-5)
 * @returns {string} CSS classes
 */
export const getPriorityClasses = (priority) =>
  PRIORITY_STYLES[priority] || PRIORITY_STYLES[3];

/**
 * Get status icon component for a status
 * @param {string} status - Status string
 * @returns {React.Component} Icon component
 */
export const getStatusIcon = (status) => {
  const iconClass = "w-4 h-4";
  switch (status) {
    case 'passed':
    case 'operational':
    case 'connected':
      return <CheckCircle2 className={iconClass} />;
    case 'failed':
    case 'disconnected':
      return <XCircle className={iconClass} />;
    case 'running':
    case 'testing':
      return <Clock className={`${iconClass} animate-spin`} />;
    case 'mock':
      return <AlertTriangle className={iconClass} />;
    default:
      return <WifiOff className={iconClass} />;
  }
};

/**
 * Format timestamp for display
 * @param {number} timestamp - Unix timestamp
 * @returns {string} Formatted time string
 */
export const formatTimestamp = (timestamp) =>
  new Date(timestamp).toLocaleTimeString();

/**
 * Truncate text with ellipsis
 * @param {string} text - Text to truncate
 * @param {number} maxLength - Maximum length
 * @returns {string} Truncated text
 */
export const truncateText = (text, maxLength = 50) =>
  text.length > maxLength ? `${text.substring(0, maxLength)}...` : text;

/**
 * Check if an object is empty
 * @param {Object} obj - Object to check
 * @returns {boolean} True if empty
 */
export const isEmpty = (obj) =>
  obj == null || Object.keys(obj).length === 0;

/**
 * Get initials from a string
 * @param {string} text - Text to get initials from
 * @returns {string} Initials
 */
export const getInitials = (text) =>
  text.split(' ').map(word => word.charAt(0)).join('').toUpperCase().slice(0, 2);

/**
 * Debounce function calls
 * @param {Function} func - Function to debounce
 * @param {number} wait - Wait time in ms
 * @returns {Function} Debounced function
 */
export const debounce = (func, wait) => {
  let timeout;
  return (...args) => {
    clearTimeout(timeout);
    timeout = setTimeout(() => func.apply(this, args), wait);
  };
};

/**
 * Calculate percentage and format
 * @param {number} value - Current value
 * @param {number} total - Total value
 * @returns {string} Formatted percentage
 */
export const formatPercentage = (value, total) =>
  total > 0 ? `${Math.round((value / total) * 100)}%` : '0%';
