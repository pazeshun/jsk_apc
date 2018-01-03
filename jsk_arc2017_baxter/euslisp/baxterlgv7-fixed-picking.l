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

(defun grasp-with-avoiding-extra-obj ()
  (let ((finger-av (send *ri* :get-real-finger-av :larm)))
    (while (< (aref finger-av (- (length finger-av) 1)) 90)
      (while (> (max (send *ri* :get-proximity :larm :right) (send *ri* :get-proximity :larm :left))
                (send *ri* :get-val 'proximity-threshold-))
        (if (send *ri* :hand-interpolatingp :larm)
          (send *ri* :cancel-move-hand :larm))
        (unix::usleep 1000))
      (if (not (send *ri* :hand-interpolatingp :larm))
        (send *ri* :move-hand :larm
              (send *baxter* :hand-grasp-pose :larm :cylindrical) 1000 :wait nil))
      (unix::usleep 1000)
      (setq finger-av (send *ri* :get-real-finger-av :larm))
      )
    ;; wait for :interpolatingp
    (dotimes (x 100)
      (if (send *ri* :hand-interpolatingp :larm) (return))
      (unix::usleep 1000))
    (while (send *ri* :hand-interpolatingp :larm) (unix::usleep 1000))
    )
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
    (send *baxter* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
    (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(0 0 40) :local) 1000)
    (send *ri* :wait-interpolation)
    (let ((palm-prox (send *ri* :get-proximity :larm :palm)) end-pos)
      (if (< palm-prox 1000)
        (progn
          (ros::ros-info "Already picked edge. Proximity: ~a" palm-prox)
          ;; release object
          (send *ri* :stop-grasp :larm)
          (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(0 0 -10) :local) 1000)
          (send *ri* :wait-interpolation)
          (unix::sleep 1))
        (progn
          (ros::ros-info "Picked center. Proximity: ~a" palm-prox)
          (send *ri* :stop-grasp :larm)
          (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(0 0 -10) :local) 1000)
          (send *ri* :wait-interpolation)
          (unix::sleep 3)
          (send *ri* :calib-proximity-threshold :larm)
          (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(100 0 0) :world)
                5000 (send *ri* :get-arm-controller :larm) 0)
          ;; wait for :interpolatingp
          (dotimes (x 100)
            (if (send *ri* :interpolatingp) (return))
            (unix::usleep 1000))
          (while (send *ri* :interpolatingp)
            (setq palm-prox (send *ri* :get-proximity :larm :palm))
            (when (< palm-prox -100)  ;; very unstable value
              (ros::ros-info "Detected edge. Cancel angle vector. Proximity: ~a" palm-prox)
              (send *ri* :cancel-angle-vector
                    :controller-type (send *ri* :get-arm-controller :larm)))
            (when (> palm-prox 700)  ;; very unstable value
              (ros::ros-info "Collide to object. Cancel angle vector. Proximity: ~a" palm-prox)
              (send *ri* :cancel-angle-vector
                    :controller-type (send *ri* :get-arm-controller :larm)))
            (unix::usleep 1000))
          (send *baxter* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))))
      ;; lift up gripper to rotate fingers
      (setq end-pos (copy-object (send *baxter* :larm :end-coords :worldpos)))
      (setf (elt end-pos 2) (aref (send init-coords :worldpos) 2))
      (send *ri* :angle-vector
            (send *baxter* :larm :inverse-kinematics
                  (make-coords :pos end-pos :rpy (car (send init-coords :rpy-angle))))
            2000)
      (send *ri* :wait-interpolation)
      (send *ri* :start-grasp :larm)
      ;; rotate fingers
      (send *ri* :move-hand :larm
            (send *baxter* :hand-grasp-pre-pose :larm :cylindrical) 1000)
      ;; grasp again
      (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(0 0 -250) :local)
            3000 (send *ri* :get-arm-controller :larm) 0)
      (send *ri* :wait-interpolation-until :larm :grasp)
      ;; lift up
      (send *ri* :angle-vector (send *baxter* :larm :inverse-kinematics init-coords) 8000)
      (grasp-with-avoiding-extra-obj)
      (send *ri* :wait-interpolation)
      )
    )
  )
