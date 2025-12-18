import { describe, it, expect } from 'vitest'
import { render, screen } from '@testing-library/react'
import { StatusBadge } from './StatusBadge'

describe('StatusBadge', () => {
  it('renders with operational status', () => {
    render(<StatusBadge status="operational" />)
    const badge = screen.getByText('Operational')
    expect(badge).toBeInTheDocument()
    expect(badge).toHaveClass('text-green-600', 'bg-green-100')
  })

  it('renders with warning status', () => {
    render(<StatusBadge status="warning" />)
    const badge = screen.getByText('Warning')
    expect(badge).toBeInTheDocument()
    expect(badge).toHaveClass('text-gray-600', 'bg-gray-100')
  })

  it('renders with error status', () => {
    render(<StatusBadge status="error" />)
    const badge = screen.getByText('Error')
    expect(badge).toBeInTheDocument()
    expect(badge).toHaveClass('text-gray-600', 'bg-gray-100')
  })

  it('renders with unknown status', () => {
    render(<StatusBadge status="unknown" />)
    const badge = screen.getByText('Unknown')
    expect(badge).toBeInTheDocument()
    expect(badge).toHaveClass('text-gray-600', 'bg-gray-100')
  })
})
