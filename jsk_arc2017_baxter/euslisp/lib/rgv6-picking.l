(require "package://jsk_2016_01_baxter_apc/euslisp/jsk_2016_01_baxter_apc/baxterrgv5-interface.l")

(defmethod jsk_2016_01_baxter_apc::baxterrgv5-interface
  (:get-finger-flexion
    (arm side)
    (send (one-shot-subscribe
            (format nil "/gripper_front/limb/~a/~a_finger_flex/state"
                    (arm2str arm) (string-downcase (symbol-name side)))
            std_msgs::UInt16)
          :data)
    )
  (:wait-interpolation-until-finger-flexion
    (arm)
    (when (send self :simulation-modep)
      (return-from :wait-interpolation-until-finger-flexion (send self :wait-interpolation))
      )
    (let ((r_init (send self :get-finger-flexion arm :r))
          (l_init (send self :get-finger-flexion arm :l)))
      (ros::ros-info
        "[:wait-interpolation-until-finger-flexion] Initial flex:  r: ~a l: ~a"
        r_init l_init)
      (while (send self :interpolatingp)
             (if (or (> (send self :get-finger-flexion arm :r) (+ r_init 15))
                     (> (send self :get-finger-flexion arm :l) (+ l_init 15)))
               (progn
                 (send self :cancel-angle-vector)
                 (ros::ros-info
                   "[:wait-interpolation-until-finger-flexion] Cancel angle vector: ~a"
                   arm)
                 ))))
    )
  )

(setq *ec-above-bin* (make-coords :pos #f(700 120 0) :rpy (float-vector (deg2rad 90) 0 0)))

(defun demo-init ()
  (jsk_2016_01_baxter_apc::baxterrgv5-init)
  (send *ri* :gripper-servo-on)
  (send *ri* :angle-vector (send *baxter* :fold-pose-back))
  (send *ri* :move-hand :rarm
        (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
  (send *ri* :move-hand :rarm
        (send *baxter* :hand-grasp-pre-pose :rarm :opposed) 1500)
  (send *ri* :wait-interpolation)
  (send *ri* :calib-pressure-threshold :rarm)
  (objects (list *baxter*))
  (let (avs)
    (pushback (send *baxter* :avoid-shelf-pose :rarm :f) avs)
    (send *baxter* :rotate-gripper :rarm 0 :relative nil)
    (pushback (send *baxter* :rarm :inverse-kinematics *ec-above-bin*) avs)
    (send *ri* :angle-vector-sequence avs :fast nil 0 :scale 3.0)
    )
  )

(defun suction-cylindrical ()
  (send *ri* :angle-vector
        (send *baxter* :rarm :inverse-kinematics *ec-above-bin*)
        :fast nil 0 :scale 15.0)
  (send *ri* :wait-interpolation)
  (send *ri* :move-hand :rarm
        (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
  (send *ri* :move-hand :rarm
        (send *baxter* :hand-grasp-pose :rarm :cylindrical :angle 90) 1500)
  (send *ri* :angle-vector
        (send *baxter* :rarm :move-end-pos #f(0 50 0) :world)
        :fast nil 0 :scale 3.0)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector
        (send *baxter* :rarm :move-end-pos #f(0 0 -300) :world)
        :fast nil 0 :scale 30.0)
  (send *ri* :wait-interpolation-until-finger-flexion :rarm)
  (send *ri* :move-hand :rarm
        (send *baxter* :hand-grasp-pose :rarm :cylindrical :angle 120) 1500)
  (send *baxter* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector
        (send *baxter* :rarm :move-end-pos #f(0 0 20) :world)
        :fast nil 0 :scale 30.0)
  (send *ri* :wait-interpolation)
  (send *ri* :move-hand :rarm
        (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :rarm)
  (send *ri* :angle-vector
        (send *baxter* :slide-gripper :rarm 80 :relative nil)
        :fast nil 0 :scale 3.0)
  (send *ri* :wait-interpolation-until-grasp :rarm)
  (send *ri* :angle-vector
        (send *baxter* :slide-gripper :rarm 0 :relative nil)
        :fast nil 0 :scale 3.0)
  (send *ri* :wait-interpolation)
  (send *ri* :move-hand :rarm
        (send *baxter* :hand-grasp-pose :rarm :cylindrical) 1500)
  (send *ri* :angle-vector
        (send *baxter* :rarm :inverse-kinematics *ec-above-bin*)
        :fast nil 0 :scale 30.0)
  (send *ri* :wait-interpolation)
  (unless (y-or-n-p "Release object?") (return-from suction-cylindrical nil))
  (send *ri* :angle-vector
        (send *baxter* :rarm :move-end-pos #f(0 0 -150) :world)
        :fast nil 0 :scale 15.0)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp)
  (send *ri* :move-hand :rarm
        (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
  (unix::sleep 2)
  (send *ri* :angle-vector
        (send *baxter* :rarm :inverse-kinematics *ec-above-bin*)
        :fast nil 0 :scale 15.0)
  (send *ri* :wait-interpolation)
  )

(defun suction-opposed ()
  (let ((ec (send *ec-above-bin* :copy-worldcoords)))
    (send ec :rotate (deg2rad -90) :z :world)
    (send *ri* :angle-vector
          (send *baxter* :rarm :inverse-kinematics ec)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pose :rarm :cylindrical :angle 140) 1500)
    (unix::sleep 1)
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 -400) :world)
          :fast nil 0 :scale 30.0)
    (send *ri* :wait-interpolation-until-finger-flexion :rarm)
    (send *baxter* :angle-vector (send *ri* :state :potentio-vector))
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 50) :world)
          :fast nil 0 :scale 30.0)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :opposed) 1500)
    (send *ri* :start-grasp :rarm)
    (send *ri* :angle-vector
          (send *baxter* :slide-gripper :rarm 120 :relative nil)
          :fast nil 0 :scale 3.0)
    (send *ri* :wait-interpolation-until-grasp :rarm)
    (send *baxter* :angle-vector (send *ri* :state :potentio-vector))
    (setq ec (send *baxter* :rarm :end-coords :copy-worldcoords))
    (send *baxter* :slide-gripper :rarm 0 :relative nil)
    (send *ri* :angle-vector
          (send *baxter* :rarm :inverse-kinematics ec)
          :fast nil 0 :scale 3.0)
    (send *ri* :wait-interpolation)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pose :rarm :opposed) 1500)
    (setq ec (send *ec-above-bin* :copy-worldcoords))
    (send ec :rotate (deg2rad -90) :z :world)
    (send *ri* :angle-vector
          (send *baxter* :rarm :inverse-kinematics ec)
          :fast nil 0 :scale 30.0)
    (send *ri* :wait-interpolation)
    (unless (y-or-n-p "Release object?") (return-from suction-opposed nil))
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 -150) :world)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :opposed) 1500)
    (unix::sleep 2)
    (send *ri* :angle-vector
          (send *baxter* :rarm :inverse-kinematics ec)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    )
  )
