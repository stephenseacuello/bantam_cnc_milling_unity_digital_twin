import { useState, useEffect, useRef, useCallback } from 'react';
import ROSLIB from 'roslib';
import { ROSBRIDGE_URL } from '../utils/rosTopics';

/**
 * Custom React hook for managing a ROS2 WebSocket connection via rosbridge.
 *
 * Provides:
 *  - Automatic connection with configurable auto-reconnect
 *  - Topic subscription helper
 *  - Service call helper
 *  - Connection status tracking
 *
 * @param {string} [url] - WebSocket URL (defaults to ROSBRIDGE_URL constant)
 * @param {object} [options]
 * @param {boolean} [options.autoReconnect=true]  - Reconnect on disconnect
 * @param {number}  [options.reconnectDelay=3000] - Delay between reconnect attempts (ms)
 * @returns {{ ros, connected, error, subscribe, callService }}
 */
export default function useROS(url, options = {}) {
  const {
    autoReconnect = true,
    reconnectDelay = 3000,
  } = options;

  const resolvedUrl = url || ROSBRIDGE_URL;

  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(null);

  const rosRef = useRef(null);
  const subscriptionsRef = useRef([]);
  const reconnectTimerRef = useRef(null);
  const mountedRef = useRef(true);

  // ------------------------------------------------------------------
  // Connect / reconnect logic
  // ------------------------------------------------------------------
  const connect = useCallback(() => {
    // Tear down any previous instance
    if (rosRef.current) {
      try { rosRef.current.close(); } catch (_) { /* ignore */ }
    }

    const ros = new ROSLIB.Ros({ url: resolvedUrl });

    ros.on('connection', () => {
      if (!mountedRef.current) return;
      setConnected(true);
      setError(null);
      console.info('[useROS] Connected to', resolvedUrl);
    });

    ros.on('error', (err) => {
      if (!mountedRef.current) return;
      setError(err?.message || 'WebSocket error');
      console.warn('[useROS] Error:', err);
    });

    ros.on('close', () => {
      if (!mountedRef.current) return;
      setConnected(false);
      console.info('[useROS] Connection closed');

      if (autoReconnect && mountedRef.current) {
        reconnectTimerRef.current = setTimeout(() => {
          if (mountedRef.current) {
            console.info('[useROS] Attempting reconnect...');
            connect();
          }
        }, reconnectDelay);
      }
    });

    rosRef.current = ros;
  }, [resolvedUrl, autoReconnect, reconnectDelay]);

  // ------------------------------------------------------------------
  // Lifecycle
  // ------------------------------------------------------------------
  useEffect(() => {
    mountedRef.current = true;
    connect();

    return () => {
      mountedRef.current = false;
      clearTimeout(reconnectTimerRef.current);

      // Unsubscribe from all active topics
      subscriptionsRef.current.forEach((sub) => {
        try { sub.unsubscribe(); } catch (_) { /* ignore */ }
      });
      subscriptionsRef.current = [];

      if (rosRef.current) {
        try { rosRef.current.close(); } catch (_) { /* ignore */ }
        rosRef.current = null;
      }
    };
  }, [connect]);

  // ------------------------------------------------------------------
  // Subscribe to a ROS topic
  // ------------------------------------------------------------------
  /**
   * Subscribe to a ROS2 topic via rosbridge.
   *
   * @param {string}   topicName    - Fully-qualified topic name
   * @param {string}   messageType  - ROS message type (e.g. "miracle_msgs/msg/MachineState")
   * @param {function} callback     - Invoked with each incoming message
   * @param {object}   [opts]       - Extra ROSLIB.Topic options (throttle_rate, etc.)
   * @returns {ROSLIB.Topic} The topic instance (call .unsubscribe() to stop)
   */
  const subscribe = useCallback((topicName, messageType, callback, opts = {}) => {
    if (!rosRef.current) {
      console.warn('[useROS] Cannot subscribe, ROS not initialised');
      return null;
    }

    const topic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topicName,
      messageType,
      throttle_rate: 200, // default 5 Hz max to keep UI responsive
      ...opts,
    });

    topic.subscribe(callback);
    subscriptionsRef.current.push(topic);
    return topic;
  }, []);

  // ------------------------------------------------------------------
  // Call a ROS service
  // ------------------------------------------------------------------
  /**
   * Call a ROS2 service via rosbridge.
   *
   * @param {string} serviceName - Fully-qualified service name
   * @param {string} serviceType - ROS service type
   * @param {object} request     - Service request payload
   * @returns {Promise<object>}  - Resolves with the service response
   */
  const callService = useCallback((serviceName, serviceType, request = {}) => {
    return new Promise((resolve, reject) => {
      if (!rosRef.current) {
        return reject(new Error('ROS not connected'));
      }

      const service = new ROSLIB.Service({
        ros: rosRef.current,
        name: serviceName,
        serviceType,
      });

      const req = new ROSLIB.ServiceRequest(request);

      service.callService(req, resolve, (err) => {
        reject(new Error(err));
      });
    });
  }, []);

  return {
    ros: rosRef.current,
    connected,
    error,
    subscribe,
    callService,
  };
}
