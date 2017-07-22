#!/usr/bin/env roseus

(require "package://jsk_arc2017_baxter/euslisp/lib/arc-interface.l")

(ros::load-ros-manifest "jsk_arc2017_baxter")

(jsk_arc2017_baxter::arc-init :ctype :rarm-controller)

(defun test (&key (arm :rarm))
  (let (prev-av next-av gripper-x)
    (setq prev-av (send *baxter* :angle-vector))
    ;; :pick-object-with-movable-region
    (setq next-av
          ;; end-coords #f(600 -700 0)
          (send *baxter* :rarm :angle-vector
                #f(-4.768 -49.2085 25.2526 50.688 -16.1874 91.2884 -30.4901 0.0 0.0)))
    (setq gripper-x (send *baxter* arm :gripper-x :joint-angle))
    (send *baxter* :angle-vector prev-av)
    ;; First, move prismatic joint to target position
    (send *ri* :angle-vector-raw
          (send *baxter* :slide-gripper arm gripper-x :relative nil)
          :fast (send *ri* :get-arm-controller arm) 0 :scale 5.0)
    (send *ri* :wait-interpolation)
    ;; Fold fingers to avoid collision
    (send *ri* :move-hand arm
          (send *baxter* :hand-grasp-pre-pose arm :cylindrical) 1000)
    (send *ri* :move-hand arm
          (send *baxter* :hand-grasp-pose arm :cylindrical :angle 110) 1000)
    ;; Move whole arm to target pose
    (send *baxter* :rarm :angle-vector next-av)
    (send *ri* :angle-vector-raw (send *baxter* :angle-vector) 3000)
    (send *ri* :wait-interpolation))

  ;; :try-to-pinch-object
  (send *ri* :move-hand arm (send *baxter* :hand-grasp-pose arm :cylindrical :angle 40) 1000)
  (send *ri* :start-grasp arm)
  ;; end-coords #f(600 -700 -300)
  (send *baxter* :rarm :angle-vector
        #f(-5.75614 -30.0687 24.2855 64.5333 -24.212 60.2145 -25.4255 0.0 0.0))
  (send *ri* :angle-vector-raw (send *baxter* :angle-vector) 3000)
  (send *ri* :wait-interpolation-until arm
        :grasp :finger-flexion :finger-loaded :prismatic-loaded :finger-proximity)
  (send *baxter* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
  (send *baxter* arm :move-end-pos #f(0 0 -40) :world)
  (send *ri* :angle-vector-raw
        (send *baxter* :slide-gripper arm 120 :relative nil)
        3000 (send *ri* :get-arm-controller arm) 0)
  (send *ri* :wait-interpolation-until arm :grasp :prismatic-loaded)
  (send *baxter* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
  ;; stop suction to bend suction finger
  (send *ri* :stop-grasp arm)
  (send *ri* :start-grasp arm :pinch)
  ;; start suction to help pinch
  (send *ri* :start-grasp arm)
  (unix::sleep 1)
  (send *baxter* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
  ;; lift object
  (let (pre-coords)
    (setq pre-coords (send (send *baxter* arm :end-coords) :copy-worldcoords))
    (send pre-coords :translate #f(0 0 200) :world)
    (setq av (send *baxter* arm :inverse-kinematics pre-coords :rotation-axis t))
    (if (null av)
        (setq av
              (send *baxter* arm :inverse-kinematics pre-coords :rotation-axis :z)))
    (send *ri* :angle-vector-raw av 3000 (send *ri* :get-arm-controller arm :gripper nil) 0)
    (send *ri* :wait-interpolation))
  ;; slide gripper to grasp tightly
  (send *baxter* :rotate-gripper arm -90 :relative nil)
  (send *baxter* :slide-gripper arm 0 :relative nil)
  (send *ri* :angle-vector-raw (send *baxter* :angle-vector)
        :fast (send *ri* :get-arm-controller arm) 0 :scale 3.0)
  (send *ri* :wait-interpolation)
  (send *baxter* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))

  (unix::sleep 3)
  (send *ri* :stop-grasp)
  (send *ri* :move-hand :rarm (send *baxter* :hand :rarm :angle-vector #f(90 0)) 1000))
