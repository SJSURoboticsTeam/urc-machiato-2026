import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import {
  getStatusClasses,
  getPriorityClasses,
  getStatusIcon,
  truncateText,
  formatTimestamp,
  formatPercentage,
  getInitials,
  isEmpty,
  debounce,
} from './uiUtils.jsx'

describe('UI Utils', () => {
  describe('getStatusClasses', () => {
    it('returns correct classes for operational status', () => {
      expect(getStatusClasses('operational')).toBe('text-green-600 bg-green-100')
    })

    it('returns correct classes for warning status', () => {
      expect(getStatusClasses('warning')).toBe('text-gray-600 bg-gray-100')
    })

    it('returns correct classes for error status', () => {
      expect(getStatusClasses('error')).toBe('text-gray-600 bg-gray-100')
    })

    it('returns default classes for unknown status', () => {
      expect(getStatusClasses('unknown')).toBe('text-gray-600 bg-gray-100')
    })
  })

  describe('getPriorityClasses', () => {
    it('returns correct classes for high priority (1)', () => {
      expect(getPriorityClasses(1)).toBe('bg-red-100 text-red-800')
    })

    it('returns correct classes for medium priority (3)', () => {
      expect(getPriorityClasses(3)).toBe('bg-green-100 text-green-800')
    })

    it('returns correct classes for low priority (5)', () => {
      expect(getPriorityClasses(5)).toBe('bg-purple-100 text-purple-800')
    })
  })

  describe('truncateText', () => {
    it('returns original text if shorter than max length', () => {
      expect(truncateText('Hello', 10)).toBe('Hello')
    })

    it('truncates text and adds ellipsis if longer than max length', () => {
      expect(truncateText('Hello World', 5)).toBe('Hello...')
    })

    it('returns empty string for empty input', () => {
      expect(truncateText('', 10)).toBe('')
    })
  })

  describe('formatTimestamp', () => {
    it('formats timestamp correctly', () => {
      const timestamp = new Date('2024-01-15T10:30:00Z').getTime()
      const result = formatTimestamp(timestamp)
      expect(typeof result).toBe('string')
      expect(result.length).toBeGreaterThan(0)
      expect(/\d/.test(result)).toBe(true)
    })
  })

  describe('formatPercentage', () => {
    it('formats percentage correctly', () => {
      expect(formatPercentage(85, 100)).toBe('85%')
      expect(formatPercentage(1, 1)).toBe('100%')
      expect(formatPercentage(0, 100)).toBe('0%')
      expect(formatPercentage(50, 0)).toBe('0%') // division by zero case
    })
  })

  describe('getInitials', () => {
    it('returns correct initials for two words', () => {
      expect(getInitials('John Doe')).toBe('JD')
    })

    it('returns correct initials for single word', () => {
      expect(getInitials('John')).toBe('J')
    })

    it('handles empty string', () => {
      expect(getInitials('')).toBe('')
    })

    it('handles multiple spaces', () => {
      expect(getInitials('John  Doe')).toBe('JD')
    })
  })

  describe('getStatusIcon', () => {
    it('returns icon for passed', () => {
      const icon = getStatusIcon('passed')
      expect(icon).toBeTruthy()
      expect(icon.props?.className).toContain('w-4 h-4')
    })
    it('returns icon for operational', () => {
      expect(getStatusIcon('operational')).toBeTruthy()
    })
    it('returns icon for connected', () => {
      expect(getStatusIcon('connected')).toBeTruthy()
    })
    it('returns icon for failed', () => {
      expect(getStatusIcon('failed')).toBeTruthy()
    })
    it('returns icon for disconnected', () => {
      expect(getStatusIcon('disconnected')).toBeTruthy()
    })
    it('returns icon for running', () => {
      const icon = getStatusIcon('running')
      expect(icon).toBeTruthy()
      expect(icon.props?.className).toContain('animate-spin')
    })
    it('returns icon for testing', () => {
      expect(getStatusIcon('testing')).toBeTruthy()
    })
    it('returns icon for mock', () => {
      expect(getStatusIcon('mock')).toBeTruthy()
    })
    it('returns default icon for unknown status', () => {
      expect(getStatusIcon('unknown')).toBeTruthy()
      expect(getStatusIcon('')).toBeTruthy()
    })
  })

  describe('isEmpty', () => {
    it('returns true for null', () => {
      expect(isEmpty(null)).toBe(true)
    })
    it('returns true for undefined', () => {
      expect(isEmpty(undefined)).toBe(true)
    })
    it('returns true for empty object', () => {
      expect(isEmpty({})).toBe(true)
    })
    it('returns false for non-empty object', () => {
      expect(isEmpty({ a: 1 })).toBe(false)
    })
  })

  describe('debounce', () => {
    beforeEach(() => {
      vi.useFakeTimers()
    })
    afterEach(() => {
      vi.useRealTimers()
    })
    it('invokes function after wait', () => {
      const fn = vi.fn()
      const debounced = debounce(fn, 100)
      debounced('a')
      expect(fn).not.toHaveBeenCalled()
      vi.advanceTimersByTime(100)
      expect(fn).toHaveBeenCalledTimes(1)
      expect(fn).toHaveBeenCalledWith('a')
    })
    it('resets timer on repeated calls', () => {
      const fn = vi.fn()
      const debounced = debounce(fn, 100)
      debounced(1)
      vi.advanceTimersByTime(50)
      debounced(2)
      vi.advanceTimersByTime(50)
      expect(fn).not.toHaveBeenCalled()
      vi.advanceTimersByTime(50)
      expect(fn).toHaveBeenCalledTimes(1)
      expect(fn).toHaveBeenCalledWith(2)
    })
  })
})
