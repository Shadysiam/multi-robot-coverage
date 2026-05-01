import { useState, useEffect, useRef, useCallback } from 'react'
import ROSLIB from 'roslib'

/**
 * Maintains a single ROSLIB.Ros connection and exposes a helper
 * for subscribing to topics with automatic cleanup.
 */
export function useRos(url) {
  const [status, setStatus] = useState('disconnected') // 'connecting' | 'connected' | 'disconnected'
  const rosRef = useRef(null)

  useEffect(() => {
    setStatus('connecting')
    const ros = new ROSLIB.Ros({ url })
    rosRef.current = ros

    ros.on('connection', () => setStatus('connected'))
    ros.on('close',      () => setStatus('disconnected'))
    ros.on('error',      () => setStatus('disconnected'))

    // Auto-reconnect every 3 s when disconnected
    const reconnect = setInterval(() => {
      if (rosRef.current && !rosRef.current.isConnected) {
        try { rosRef.current.connect(url) } catch (_) {}
      }
    }, 3000)

    return () => {
      clearInterval(reconnect)
      ros.close()
    }
  }, [url])

  /**
   * Subscribe to a ROS topic.  Returns an unsubscribe function.
   * Safe to call before the connection is established — roslibjs queues it.
   */
  const subscribe = useCallback((name, messageType, callback) => {
    if (!rosRef.current) return () => {}
    const topic = new ROSLIB.Topic({ ros: rosRef.current, name, messageType })
    topic.subscribe(callback)
    return () => topic.unsubscribe()
  }, [])

  return { ros: rosRef.current, status, subscribe }
}
