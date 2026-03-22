import React from 'react';
import { STATUS_STYLES } from '../../constants/uiConstants';

/**
 * Reusable Status Badge Component
 * Consistent status display across the application
 */
export const StatusBadge = ({ status, size = 'sm', variant = 'filled' }) => {
  const baseClasses = "inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium";

  let statusClass = STATUS_STYLES[status] || STATUS_STYLES.unknown;

  if (variant === 'outline') {
    // Convert filled styles to outline styles
    statusClass = statusClass
      .replace('bg-', 'border-2 border-')
      .replace('text-', 'text-')
      .replace(/\s+bg-[^-]+/g, ' bg-transparent');
  }

  return (
    <span className={`${baseClasses} ${statusClass}`}>
      {status.charAt(0).toUpperCase() + status.slice(1)}
    </span>
  );
};
