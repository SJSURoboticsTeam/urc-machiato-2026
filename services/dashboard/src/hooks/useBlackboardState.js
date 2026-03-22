import { useState, useCallback, useEffect } from 'react';
import ROSLIB from '../utils/rosbridge';
import { BLACKBOARD_SERVICES, SERVICE_TYPES } from '../config/rosTopics';
import { BLACKBOARD_DOMAINS } from '../components/debugging/constants';

/**
 * Fetch all blackboard keys via get_value service (batch by key list).
 * @param {object} ros - ROS connection (from useROS)
 * @param {boolean} enabled - Whether to poll (e.g. when tab is active)
 * @returns {[object, boolean, function]} [state, loading, refresh]
 */
export function useBlackboardState(ros, enabled = true) {
  const [state, setState] = useState({});
  const [loading, setLoading] = useState(false);
  const allKeys = Object.values(BLACKBOARD_DOMAINS).flatMap((d) => d.keys);

  const refresh = useCallback(() => {
    if (!ros || !enabled) return;
    setLoading(true);
    const serviceType = SERVICE_TYPES.GET_BLACKBOARD_VALUE || 'autonomy_interfaces/srv/GetBlackboardValue';
    const client = new ROSLIB.Service({
      ros,
      name: BLACKBOARD_SERVICES.GET_VALUE,
      serviceType
    });
    const results = {};
    let done = 0;
    const checkDone = () => {
      done += 1;
      if (done >= allKeys.length) {
        setState((prev) => ({ ...prev, ...results }));
        setLoading(false);
      }
    };
    allKeys.forEach((key) => {
      client.callService(
        { key, value_type: '' },
        (res) => {
          if (res?.success && res?.value != null) {
            let v = res.value;
            if (res.value_type === 'bool') v = v === 'true';
            else if (res.value_type === 'int') v = parseInt(v, 10);
            else if (res.value_type === 'double') v = parseFloat(v, 10);
            results[key] = v;
          }
          checkDone();
        },
        checkDone
      );
    });
    if (allKeys.length === 0) setLoading(false);
  }, [ros, enabled]);

  useEffect(() => {
    if (!enabled || !ros) return;
    refresh();
    const id = setInterval(refresh, 2000);
    return () => clearInterval(id);
  }, [enabled, ros, refresh]);

  return [state, loading, refresh];
}
