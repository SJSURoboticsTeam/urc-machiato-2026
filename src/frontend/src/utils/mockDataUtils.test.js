import { describe, it, expect, vi } from 'vitest'
import {
  generateMockGPS,
  generateMockIMU,
  generateMockBattery,
  generateMessageDelay,
  shouldTestSucceed
} from './mockDataUtils'

// Mock Math.random for predictable tests
const mockMath = Object.create(global.Math)
mockMath.random = vi.fn()
global.Math = mockMath

describe('Mock Data Utils', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  describe('generateMockGPS', () => {
    it('returns GPS object with correct structure', () => {
      mockMath.random.mockReturnValue(0.5)
      const gps = generateMockGPS()

      expect(gps).toHaveProperty('latitude')
      expect(gps).toHaveProperty('longitude')
      expect(gps).toHaveProperty('altitude')
      expect(typeof gps.latitude).toBe('number')
      expect(typeof gps.longitude).toBe('number')
      expect(typeof gps.altitude).toBe('number')
    })

    it('generates GPS within expected ranges', () => {
      mockMath.random.mockReturnValue(0.5)
      const gps = generateMockGPS()

      expect(gps.latitude).toBeGreaterThanOrEqual(38.35) // GPS_LATITUDE_BASE - GPS_POSITION_VARIANCE
      expect(gps.latitude).toBeLessThanOrEqual(38.45) // GPS_LATITUDE_BASE + GPS_POSITION_VARIANCE
      expect(gps.longitude).toBeGreaterThanOrEqual(-110.85) // GPS_LONGITUDE_BASE - GPS_POSITION_VARIANCE
      expect(gps.longitude).toBeLessThanOrEqual(-110.75) // GPS_LONGITUDE_BASE + GPS_POSITION_VARIANCE
      expect(gps.altitude).toBeGreaterThanOrEqual(1490) // GPS_ALTITUDE_BASE - GPS_ALTITUDE_VARIANCE
      expect(gps.altitude).toBeLessThanOrEqual(1510) // GPS_ALTITUDE_BASE + GPS_ALTITUDE_VARIANCE
    })
  })

  describe('generateMockIMU', () => {
    it('returns IMU object with correct structure', () => {
      mockMath.random.mockReturnValue(0.5)
      const imu = generateMockIMU()

      expect(imu).toHaveProperty('accel_x')
      expect(imu).toHaveProperty('accel_y')
      expect(imu).toHaveProperty('accel_z')
      expect(imu).toHaveProperty('gyro_x')
      expect(imu).toHaveProperty('gyro_y')
      expect(imu).toHaveProperty('gyro_z')
      expect(imu).toHaveProperty('temperature')
      expect(typeof imu.accel_x).toBe('number')
      expect(typeof imu.gyro_x).toBe('number')
    })

    it('generates IMU values within expected ranges', () => {
      mockMath.random.mockReturnValue(0.5)
      const imu = generateMockIMU()

      expect(imu.temperature).toBeGreaterThanOrEqual(22.5) // IMU_TEMPERATURE_BASE - 2.5
      expect(imu.temperature).toBeLessThanOrEqual(27.5) // IMU_TEMPERATURE_BASE + 2.5
    })
  })

  describe('generateMockBattery', () => {
    it('returns battery object with correct structure', () => {
      mockMath.random.mockReturnValue(0.5)
      const battery = generateMockBattery()

      expect(battery).toHaveProperty('voltage')
      expect(battery).toHaveProperty('current')
      expect(battery).toHaveProperty('percentage')
      expect(typeof battery.voltage).toBe('number')
      expect(typeof battery.current).toBe('number')
      expect(typeof battery.percentage).toBe('number')
    })

    it('generates battery percentage between 0 and 100', () => {
      mockMath.random.mockReturnValue(0.5)
      const battery = generateMockBattery()

      expect(battery.percentage).toBeGreaterThanOrEqual(0)
      expect(battery.percentage).toBeLessThanOrEqual(100)
    })
  })

  describe('generateMessageDelay', () => {
    it('returns number within expected range', () => {
      mockMath.random.mockReturnValue(0.5)
      const delay = generateMessageDelay()

      expect(typeof delay).toBe('number')
      expect(delay).toBeGreaterThanOrEqual(500) // MESSAGE_DELAY_MIN
      expect(delay).toBeLessThanOrEqual(1000) // MESSAGE_DELAY_MAX
    })
  })

  describe('shouldTestSucceed', () => {
    it('returns true when random value is below success rate', () => {
      mockMath.random.mockReturnValue(0.7) // Below 0.8 default
      expect(shouldTestSucceed()).toBe(true)
    })

    it('returns false when random value is above success rate', () => {
      mockMath.random.mockReturnValue(0.9) // Above 0.8 default
      expect(shouldTestSucceed()).toBe(false)
    })

    it('accepts custom success rate', () => {
      mockMath.random.mockReturnValue(0.5) // Below 0.9 custom rate
      expect(shouldTestSucceed(0.9)).toBe(true)
    })
  })
})
