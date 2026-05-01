import { useState, useEffect, useRef, useCallback } from 'react'
// roslib is CJS — Vite's optimizeDeps pre-bundles it, but the default
// export may land as the module itself or as .default depending on the bundler.
import ROSLIBModule from 'roslib'
const ROSLIB = ROSLIBModule.default ?? ROSLIBModule

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

    // Wrap callback to catch per-message errors (prevents whole React tree crash)
    const safeCallback = (msg) => {
      try {
        callback(msg)
      } catch (err) {
        console.error(`[useRos] Error handling message on ${name}:`, err)
      }
    }

    topic.subscribe(safeCallback)
    return () => {
      try { topic.unsubscribe() } catch (_) {}
    }
  }, [])

  /**
   * Publish a single message to a ROS topic.
   */
  const publish = useCallback((name, messageType, message) => {
    if (!rosRef.current) return
    try {
      const topic = new ROSLIB.Topic({ ros: rosRef.current, name, messageType })
      topic.publish(new ROSLIB.Message(message))
    } catch (err) {
      console.error(`[useRos] Error publishing to ${name}:`, err)
    }
  }, [])

  return { ros: rosRef.current, status, subscribe, publish }
}
