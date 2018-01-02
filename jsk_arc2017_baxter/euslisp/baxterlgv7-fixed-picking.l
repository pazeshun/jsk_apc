#!/usr/bin/env roseus

(require "package://jsk_arc2017_baxter/euslisp/lib/baxterlgv7-interface.l")

(defun demo-init ()
  (jsk_arc2017_baxter::baxterlgv7-init :type :default-controller :moveit nil)
  (send *ri* :angle-vector (send *baxter* :arc-reset-pose) 10000)
  (send *ri* :wait-interpolation)
  ;; initialize fingers
  (send *ri* :move-hand :larm
        (send *baxter* :hand-grasp-pre-pose :larm :cylindrical) 1000)
  (send *ri* :move-hand :larm
        (send *baxter* :hand-grasp-pre-pose :larm :opposed) 1000)
  (send *ri* :calib-pressure-threshold :larm)
  (objects (list *baxter*)))

(defun go-initial ()
  (send *ri* :stop-grasp :larm)
  (send *ri* :angle-vector (send *baxter* :arc-reset-pose) 5000)
  (send *ri* :wait-interpolation)
  ;; initialize fingers
  (send *ri* :move-hand :larm
        (send *baxter* :hand-grasp-pre-pose :larm :cylindrical) 1000)
  (send *ri* :move-hand :larm
        (send *baxter* :hand-grasp-pre-pose :larm :opposed) 1000)
  )

(defun edge-picking-in-wide-space ()
  (let ((init-coords (make-coords :pos #f(521.756 341.054 -160) :rpy (float-vector pi 0 0))))
    (send *ri* :angle-vector (send *baxter* :larm :inverse-kinematics init-coords) 2000)
    (send *ri* :wait-interpolation)
    ;; Enable suction finger to reach bottom
    ;; (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(0 0 -40) :local) 2000)
    ;; (send *ri* :wait-interpolation)
    (send *ri* :calib-proximity-threshold :larm)
    (send *ri* :start-grasp :larm)
    (unix::sleep 1)
    (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(0 0 -250) :local)
          5000 (send *ri* :get-arm-controller :larm) 0)
    (send *ri* :wait-interpolation-until :larm :grasp)
    (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(0 0 50) :local) 1000)
    (send *ri* :wait-interpolation)
    (let ((palm-prox (send *ri* :get-proximity :larm :palm)))
      (if (< palm-prox 2000)
        (progn
          (ros::ros-info "Already picked edge. Proximity: ~a" palm-prox)
          (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(0 0 70) :local) 1000)
          (send *ri* :wait-interpolation)
          (send *ri* :move-hand :larm
                (send *baxter* :hand-grasp-pre-pose :larm :cylindrical) 1000)
          (send *ri* :move-hand :larm
                (send *baxter* :hand-grasp-pose :larm :cylindrical) 1000)
          (unix::sleep 1)
          (send *ri* :angle-vector (send *baxter* :larm :inverse-kinematics init-coords) 2000)
          (send *ri* :wait-interpolation)
          )
        (progn
          (ros::ros-info "Picked center. Proximity: ~a" palm-prox)
          (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(0 0 -20) :local) 1000)
          (send *ri* :stop-grasp :larm)
          (send *ri* :wait-interpolation)
          (unix::sleep 2)
          (send *ri* :calib-proximity-threshold :larm)
          (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(100 0 0) :world)
                5000 (send *ri* :get-arm-controller :larm) 0)
          ;; wait for :interpolatingp
          (dotimes (x 100)
            (if (send *ri* :interpolatingp) (return))
            (unix::usleep 1000))
          (while (send *ri* :interpolatingp)
            (setq palm-prox (send *ri* :get-proximity :larm :palm))
            (when (< palm-prox -800)  ;; very unstable value
              (ros::ros-info "Detected edge. Cancel angle vector. Proximity: ~a" palm-prox)
              (send *ri* :cancel-angle-vector
                    :controller-type (send *ri* :get-arm-controller :larm)))
            (unix::usleep 1000))
          ;; grasp again
          (send *ri* :start-grasp :larm)
          (unix::sleep 1)
          (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(0 0 -50) :local)
                2000 (send *ri* :get-arm-controller :larm) 0)
          (send *ri* :wait-interpolation-until :larm :grasp)
          ;; lift up
          (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(0 0 120) :local) 2000)
          (send *ri* :wait-interpolation)
          (send *ri* :move-hand :larm
                (send *baxter* :hand-grasp-pre-pose :larm :cylindrical) 1000)
          (unix::usleep 500000)
          (send *ri* :move-hand :larm
                (send *baxter* :hand-grasp-pose :larm :cylindrical) 1000)
          (unix::sleep 1)
          (send *ri* :angle-vector (send *baxter* :larm :inverse-kinematics init-coords) 2000)
          (send *ri* :wait-interpolation)
          )
        )
      )
    )
  )
