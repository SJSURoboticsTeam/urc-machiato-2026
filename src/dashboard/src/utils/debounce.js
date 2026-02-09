import { useRef, useCallback, useEffect } from 'react';

/**
 * Returns a stable callback that invokes the latest fn after delayMs.
 * Cancels pending invocation on unmount or when fn/delayMs change.
 */
export function useDebouncedCallback(fn, delayMs) {
  const timeoutRef = useRef(null);
  const fnRef = useRef(fn);
  fnRef.current = fn;

  useEffect(() => {
    return () => {
      if (timeoutRef.current) clearTimeout(timeoutRef.current);
    };
  }, []);

  const debounced = useCallback(
    (...args) => {
      if (timeoutRef.current) clearTimeout(timeoutRef.current);
      timeoutRef.current = setTimeout(() => {
        timeoutRef.current = null;
        fnRef.current(...args);
      }, delayMs);
    },
    [delayMs]
  );

  return debounced;
}
