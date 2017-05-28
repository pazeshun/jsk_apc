(require "package://jsk_2016_01_baxter_apc/euslisp/jsk_2016_01_baxter_apc/baxterrgv5-interface.l")

(defmethod jsk_2016_01_baxter_apc::baxterrgv5-interface
  (:get-finger-flexion
    (arm side)
    (send (one-shot-subscribe
            (format nil "/gripper_front/limb/~a/~a_finger_flex/state"
                    (arm2str arm) (symbol2str side))
            std_msgs::UInt16)
          :data)
    )
  (:wait-interpolation-until-finger-moves
    (arm direction)
    (when (not (or (eq direction :flexion) (eq direction :extension)))
      (error
        ":wait-interpolation-until-finger-moves direction must be :flexion or :extension~%"))
    (when (send self :simulation-modep)
      (send self :wait-interpolation)
      (return-from :wait-interpolation-until-finger-moves nil))
    (let ((r_init (send self :get-finger-flexion arm :r))
          (l_init (send self :get-finger-flexion arm :l))
          r_thre l_thre opr)
      (ros::ros-info
        "[:wait-interpolation-until-finger-moves] Initial flex:  r: ~a l: ~a"
        r_init l_init)
      (if (eq direction :flexion)
        (setq opr #'> r_thre (+ r_init 20) l_thre (+ l_init 20))
        (setq opr #'< r_thre (- r_init 30) l_thre (- l_init 30)))
      (while (send self :interpolatingp)
             (if (or (funcall opr (send self :get-finger-flexion arm :r) r_thre)
                     (funcall opr (send self :get-finger-flexion arm :l) l_thre))
               (progn
                 (send self :cancel-angle-vector)
                 (ros::ros-info
                   "[:wait-interpolation-until-finger-moves] Cancel angle vector: ~a"
                   arm)
                 (return-from :wait-interpolation-until-finger-moves t)
                 ))))
    nil)
  (:get-real-finger-roll-angle
    (arm)
    (send self :update-robot-state :wait-until-update t)
    (send robot (str2symbol (format nil "~a_gripper_finger_roll_joint" (arm2str arm)))
          :joint-angle))
  )

(setq *ec-above-bin* (make-coords :pos #f(700 -150 50) :rpy (float-vector (deg2rad 90) 0 0)))

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
  (send *baxter* :avoid-shelf-pose :rarm :f)
  (send *baxter* :rotate-gripper :rarm 0 :relative nil)
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
  (send *ri* :wait-interpolation-until-finger-moves :rarm :flexion)
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

(defun suction-cylindrical-com ()
  (let (ec)
    (send *baxter* :avoid-shelf-pose :rarm :f)
    (send *baxter* :rotate-gripper :rarm 0 :relative nil)
    (send *ri* :angle-vector
          (send *baxter* :rarm :inverse-kinematics *ec-above-bin*)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pose :rarm :cylindrical :angle 140) 1500)
    (unix::sleep 2)
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 -400) :world)
          :fast nil 0 :scale 30.0)
    (send *ri* :wait-interpolation-until-finger-moves :rarm :flexion)
    (setq ec (send *baxter* :rarm :end-coords :copy-worldcoords))
    (send *baxter* :angle-vector (send *ri* :state :potentio-vector))
    (send ec :translate
          (float-vector 0 0 (- (elt (send *baxter* :rarm :end-coords :worldpos) 2)
                               (elt (send ec :worldpos) 2)))
          :world)
    (send *baxter* :rarm :inverse-kinematics ec)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 80 0) :world)
          :fast nil 0 :scale 15.0)
    (when (send *ri* :wait-interpolation-until-finger-moves :rarm :extension)
      (send *ri* :angle-vector
            (send *baxter* :rarm :move-end-pos #f(0 -40 0) :world)
            :fast nil 0 :scale 15.0)
      (send *ri* :wait-interpolation)
      )
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 50) :world)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 -10) :world)
          :fast nil 0 :scale 15.0)
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
    (send *ri* :wait-interpolation)
    (send *baxter* :rotate-gripper :rarm 0 :relative nil)
    (send *ri* :angle-vector
          (send *baxter* :rarm :inverse-kinematics *ec-above-bin*)
          :fast :no-gripper-controller 0 :scale 30.0)
    (send *ri* :wait-interpolation)
    (unless (y-or-n-p "Release object?") (return-from suction-cylindrical nil))
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 -150) :world)
          :fast :no-gripper-controller 0 :scale 15.0)
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
  )

(defun suction-opposed ()
  (let ((ec (send *ec-above-bin* :copy-worldcoords)))
    ;; initialize
    (send ec :rotate (deg2rad -90) :z :world)
    (send *baxter* :avoid-shelf-pose :rarm :f)
    (send *baxter* :rotate-gripper :rarm 0 :relative nil)
    (send *ri* :angle-vector
          (send *baxter* :rarm :inverse-kinematics ec)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pose :rarm :cylindrical :angle 140) 1500)
    (unix::sleep 1)
    ;; lower hand until finger flexes
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 -400) :world)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation-until-finger-moves :rarm :flexion)
    (send *baxter* :angle-vector (send *ri* :state :potentio-vector))
    ;; lift hand
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 20) :world)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    ;; Move hand forward until finger extends
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(100 0 0) :local)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation-until-finger-moves :rarm :extension)
    (unix::sleep 1)
    (send *baxter* :angle-vector (send *ri* :state :potentio-vector))
    ;; lift hand
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 50) :world)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    ;; move hand backward
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(-100 0 0) :local)
          :fast nil 0 :scale 15.0)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :opposed) 1500)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp :rarm)
    ;; slide gripper until grasp
    (send *ri* :angle-vector
          (send *baxter* :slide-gripper :rarm 120 :relative nil)
          :fast nil 0 :scale 3.0)
    (send *ri* :wait-interpolation-until-grasp :rarm)
    (send *baxter* :angle-vector (send *ri* :state :potentio-vector))
    ;; slide hand
    (setq ec (send *baxter* :rarm :end-coords :copy-worldcoords))
    (send *baxter* :slide-gripper :rarm 0 :relative nil)
    (send *ri* :angle-vector
          (send *baxter* :rarm :inverse-kinematics ec)
          :fast nil 0 :scale 3.0)
    (send *ri* :wait-interpolation)
    ;; hand grasp
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pose :rarm :opposed) 1500)
    (setq ec (send *ec-above-bin* :copy-worldcoords))
    (send ec :rotate (deg2rad -90) :z :world)
    (send *ri* :angle-vector
          (send *baxter* :rarm :inverse-kinematics ec)
          :fast nil 0 :scale 15.0)
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

(defun suction-opposed-pinch-first ()
  (let ((ec (send *ec-above-bin* :copy-worldcoords)) (curr-f-roll 0) (prev-f-roll 0))
    ;; initialize
    (send ec :rotate (deg2rad -90) :z :world)
    (send *baxter* :avoid-shelf-pose :rarm :f)
    (send *baxter* :rotate-gripper :rarm 0 :relative nil)
    (send *baxter* :rarm :inverse-kinematics ec)
    (send *ri* :angle-vector
          (send *baxter* :rotate-gripper :rarm 90 :relative nil)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :opposed) 1500)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pose :rarm :opposed :angle 120) 1500)
    ;; move hand into bin
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 -100) :world)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    ;; open fingers
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :opposed) 1500)
    ;; push hand to object
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 -280) :world)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    (send *baxter* :angle-vector (send *ri* :state :potentio-vector))
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 20) :world)
          :fast nil 0 :scale 3.0)
    (send *ri* :wait-interpolation)
    ;; close fingers
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pose :rarm :opposed) 1500)
    (unix::sleep 2)
    (while t
      (setq prev-f-roll (send *ri* :get-real-finger-roll-angle :rarm))
      (send *ri* :angle-vector
            (send *baxter* :rarm :move-end-pos #f(0 0 10) :world)
            :fast nil 0 :scale 15.0)
      (send *ri* :wait-interpolation)
      (setq curr-f-roll (send *ri* :get-real-finger-roll-angle :rarm))
      (if (< (- curr-f-roll prev-f-roll) 5) (return))
      )
    ;; This is not good motion. Stop developing this.
    )
  )

(defun cylindrical ()
  (let ((ec (send *ec-above-bin* :copy-worldcoords)))
    ;; initialize
    (send ec :rotate (deg2rad 90) :x :world)
    (send *baxter* :avoid-shelf-pose :rarm :f)
    (send *baxter* :rotate-gripper :rarm -90 :relative nil)
    (send *baxter* :rarm :inverse-kinematics ec)
    (send *ri* :angle-vector
          (send *baxter* :rotate-gripper :rarm 0 :relative nil)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
    (send *ri* :angle-vector
          (send *baxter* :slide-gripper :rarm 60 :relative nil)
          :fast nil 0 :scale 3.0)
    (send *ri* :wait-interpolation)
    (send *ri* :update-robot-state :wait-until-update t)
    ;; grasp object
    (send *ri* :start-grasp :rarm)
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 -270) :world)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation-until-grasp :rarm)
    (send *baxter* :angle-vector (send *ri* :state :potentio-vector))
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pose :rarm :cylindrical) 1500)
    (send *ri* :angle-vector
          (send *baxter* :rotate-gripper :rarm -90 :relative nil)
          :fast nil 0 :scale 3.0)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector
          (send *baxter* :slide-gripper :rarm 0 :relative nil)
          :fast nil 0 :scale 3.0)
    (send *ri* :wait-interpolation)
    ;; lift object
    (setq ec (send *ec-above-bin* :copy-worldcoords))
    (send ec :rotate (deg2rad 90) :x :world)
    (send *ri* :angle-vector
          (send *baxter* :rarm :inverse-kinematics ec)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    (unless (y-or-n-p "Release object?") (return-from cylindrical nil))
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 -150) :world)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
    (unix::sleep 2)
    (send *baxter* :rarm :inverse-kinematics ec)
    (send *ri* :angle-vector
          (send *baxter* :rotate-gripper :rarm 0 :relative nil)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    )
  )

(defun suction ()
  (let ((ec (send *ec-above-bin* :copy-worldcoords)))
    ;; initialize
    (send ec :rotate (deg2rad 180) :z :world)
    (send *baxter* :avoid-shelf-pose :rarm :f)
    (send *baxter* :rotate-gripper :rarm 0 :relative nil)
    (send *ri* :angle-vector
          (send *baxter* :rarm :inverse-kinematics ec)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
    (send *ri* :angle-vector
          (send *baxter* :slide-gripper :rarm 50 :relative nil)
          :fast nil 0 :scale 3.0)
    (send *ri* :wait-interpolation)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pose :rarm :cylindrical) 1500)
    ;; suction grasp
    (send *ri* :start-grasp :rarm)
    (send *ri* :angle-vector
          (send *baxter* :rarm :move-end-pos #f(0 0 -270) :world)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation-until-grasp :rarm)
    ;; lift object
    (setq ec (send *ec-above-bin* :copy-worldcoords))
    (send ec :rotate (deg2rad 180) :z :world)
    (send *ri* :angle-vector
          (send *baxter* :rarm :inverse-kinematics ec)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    ;; hand grasp
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
    (send *baxter* :rotate-gripper :rarm -90 :relative nil)
    (send ec :locate #f(0 0 250))
    (send *ri* :angle-vector
          (send *baxter* :rarm :inverse-kinematics ec :rotation-axis :z)
          :fast nil 0 :scale 15.0)
    (send *ri* :wait-interpolation)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pose :rarm :cylindrical) 1500)
    (unless (y-or-n-p "Release object?") (return-from suction nil))
    (send *ri* :stop-grasp :rarm)
    (send *ri* :move-hand :rarm
          (send *baxter* :hand-grasp-pre-pose :rarm :cylindrical) 1500)
    )
  )
