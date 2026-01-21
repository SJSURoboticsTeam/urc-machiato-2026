import { describe, it, expect } from 'vitest'
import {
  getStatusClasses,
  getPriorityClasses,
  truncateText,
  formatTimestamp,
  formatPercentage,
  getInitials
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
      const timestamp = new Date('2024-01-15T10:30:00Z')
      expect(formatTimestamp(timestamp)).toMatch(/\d{2}:\d{2}:\d{2}/)
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
})
