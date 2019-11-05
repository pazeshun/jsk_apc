;; -*- mode: lisp;-*-

(require "package://jsk_arc2017_baxter/euslisp/lib/baxterlgv8-interface.l")

(unless (find-package "JSK_ARC2017_BAXTER")
  (make-package "JSK_ARC2017_BAXTER"))

(defclass jsk_arc2017_baxter::paper-filing-interface
  :super propertied-object
  :slots (moveit-p-
          table-cube-
          gripper-dims-
          paper-boxes-
          paper-coords-
          paper-thre-l-
          paper-thre-r-
          ))

;; conditions of finger sensors
(defcondition fingers-paper-event :slots (l-dist r-dist))
(defcondition fingers-see-paper :super fingers-paper-event)
(defcondition fingers-touch-paper :super fingers-paper-event)
(defcondition fingers-detach-paper :super fingers-paper-event)

(defmethod jsk_arc2017_baxter::paper-filing-interface
  (:init (&key (moveit nil))
    (setq moveit-p- moveit)
    (setq table-cube- nil)
    ;; dimensions of grippers
    (setq gripper-dims- (make-hash-table))
    (sethash :pad-r gripper-dims- (make-hash-table))
    (sethash :larm (gethash :pad-r gripper-dims-) 15)
    (sethash :rarm (gethash :pad-r gripper-dims-) 15)
    (setq paper-boxes- nil)
    (setq paper-coords- nil)
    (setq paper-thre-l- 25 paper-thre-r- 25))
  (:bbox->cube (bbox)
    (let* ((dims (ros::tf-point->pos (send bbox :dimensions)))
           (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2))))
      (send bx :newcoords
            (send *ri* :tf-pose->coords
                  (send bbox :header :frame_id)
                  (send bbox :pose)))
      bx))
  (:recognize-table-box
    (&key (stamp (ros::time-now)))
    (ros::ros-info "[:recognize-table-box] Recognizing table box...")
    (let ((box-topic (format nil "publish_table_boxes/output"))
          box-msg box-list bin-list)
      (setq box-msg (one-shot-subscribe box-topic
                                        jsk_recognition_msgs::BoundingBoxArray
                                        :timeout 10000
                                        :after-stamp stamp))
      (if box-msg
        (progn
          (ros::ros-info "[:recognize-table-box] Recognized table box")
          (setq table-cube- (send self :bbox->cube (car (send box-msg :boxes)))))
        (ros::ros-fatal "[:recognize-table-box] Cannot recognize table box"))))
  (:ik->cube-center
    (arm cube &key (offset #f(0 0 0)) (rpy #f(0 0 0)) (rotation-axis t)
         (use-gripper nil) (move-palm-end nil) (use-rotate-wrist-ik nil) (parallel-with :local))
    (let (cube-coords)
      (setq cube-coords (send cube :copy-worldcoords))
      (send cube-coords :translate
            (float-vector 0.0 0.0 (/ (z-of-cube cube) 2.0))
            :local)
      (send cube-coords :translate offset :local)
      (send cube-coords :rotate (aref rpy 0) :z)
      (send cube-coords :rotate (aref rpy 1) :y)
      (send cube-coords :rotate (aref rpy 2) :x)
      (when (eq parallel-with :world)
        (setq cube-coords (make-coords :pos (send cube-coords :worldpos))))
      (if use-rotate-wrist-ik
        (send *baxter* :rotate-wrist-ik arm cube-coords
              :rotation-axis rotation-axis
              :use-gripper use-gripper
              :move-palm-end move-palm-end)
        (send *baxter* arm :inverse-kinematics cube-coords
              :rotation-axis rotation-axis
              :use-gripper use-gripper
              :move-palm-end move-palm-end))))
  (:move-larm->recognition-pose ()
    (ros::ros-info "[:move-larm->recognition-pose] Moving larm to recognition pose")
    (send *baxter* :reset-pose :larm)
    (send *baxter* :rotate-gripper :larm -90 :relative nil)
    (send self :ik->cube-center :larm table-cube-
          :offset #f(30 290 250) :rpy (float-vector 0 -pi/2 pi/2)
          :use-gripper nil :use-rotate-wrist-ik t)
    (send *ri* :angle-vector-raw (send *baxter* :angle-vector) 3000
          (send *ri* :get-arm-controller :larm) 0))
  (:recognize-paper (&key (stamp (ros::time-now)) (timeout *inf*))
    (ros::ros-info "[:recognize-paper] Recognizing paper on table...")
    (let ((box-topic "left_hand_camera/cluster_indices_decomposer_paper/boxes")
          box-msg
          (com-topic "left_hand_camera/cluster_indices_decomposer_paper/centroid_pose_array")
          com-msg recognized-p)
      (ros::subscribe box-topic jsk_recognition_msgs::BoundingBoxArray
                      #'(lambda (msg)
                          (let ((st (send msg :header :stamp)))
                            (when (> (send st :to-sec) (send stamp :to-sec))
                              (setq box-msg msg)))))
      (ros::subscribe com-topic geometry_msgs::PoseArray
                      #'(lambda (msg)
                          (let ((st (send msg :header :stamp)))
                            (when (> (send st :to-sec) (send stamp :to-sec))
                              (setq com-msg msg)))))
      (while (and (not (setq recognized-p (and box-msg com-msg
                                               (> (length (send box-msg :boxes)) 0)
                                               (> (length (send com-msg :poses)) 0))))
                  (> (+ (send stamp :to-sec) timeout) (send (ros::time-now) :to-sec)))
        (unix::usleep (* 50 1000))
        (ros::spin-once))
      (when recognized-p
        (ros::ros-info "[:recognize-paper] Get first result, but waiting for stable result")
        ;; Get stable result
        (setq stamp (ros::time+ (ros::time-now) (ros::time 3)))
        (setq box-msg nil com-msg nil)
        (while (and (not (setq recognized-p (and box-msg com-msg
                                                 (> (length (send box-msg :boxes)) 0)
                                                 (> (length (send com-msg :poses)) 0))))
                    (> (+ (send stamp :to-sec) timeout) (send (ros::time-now) :to-sec)))
          (unix::usleep (* 50 1000))
          (ros::spin-once)))
      (ros::unsubscribe box-topic)
      (ros::unsubscribe com-topic)
      (if recognized-p
        (progn
          (ros::ros-info "[:recognize-paper] Get cpi msg including bbox and com")
          (setq paper-boxes- (send box-msg :boxes))
          (setq paper-coords-
                (mapcar #'(lambda (obj-pose)
                            (send *ri* :tf-pose->coords
                                  (send com-msg :header :frame_id) obj-pose))
                        (send com-msg :poses)))
          t)
        (progn
          (ros::ros-error "[:recognize-paper] Failed to get cpi msg including bbox and com")
          nil))))
  (:slide-paper ()
    (ros::ros-info "[:slide-paper] Sliding paper to grasp...")
    ;; Turn off unused proximities. Don't forget to turn on afterwords
    (send *ri* :turn-off-proximity :intensity :larm
          (list :left-root :right-root :palm-right :palm-left))
    (send *ri* :turn-off-proximity :tof :larm
          (list :left-root :right-root :palm-right :palm-left))
    (unix::sleep 1)
    (send *ri* :calib-proximity-threshold :larm)
    (send *ri* :reset-i-prox-calib :larm)
    (send *ri* :move-hand :larm
          (send *baxter* :hand-grasp-pre-pose :larm :cylindrical) 1000 :wait nil)
    (let ((paper-cube (send self :bbox->cube (car paper-boxes-)))
          contact-coords target-protrusion offset-from-edge above-coords slide-l gripper-l avs)
      (setq contact-coords (send paper-cube :copy-worldcoords))
      ;; Decide y position from paper edge
      (setq target-protrusion 50)
      (setq offset-from-edge (+ target-protrusion
                                (gethash :larm (gethash :pad-r gripper-dims-))))
      (send contact-coords :translate
            (float-vector 0 (- (/ (y-of-cube paper-cube) 2) offset-from-edge) 0)
            :local)
      ;; Decide z position from table-cube- as recognition result may be unstable
      (setf (elt (send contact-coords :worldpos) 2) (+ (elt (send table-cube- :worldpos) 2)
                                                       (/ (z-of-cube table-cube-) 2) -20))
      ;; Rotate to match end-coords
      (send contact-coords :rotate (deg2rad -90) :z)
      (setq above-coords (copy-object contact-coords))
      (send above-coords :translate #f(0 0 150))
      (setq slide-l
            (- (+ (elt (send table-cube- :worldpos) 1) (/ (y-of-cube table-cube-) 2))
               (elt (send contact-coords :worldpos) 1) (gethash :larm (gethash :pad-r gripper-dims-))
               30))
      (ros::ros-info "[:slide-paper] above-coords: ~a -> contact-coords: ~a -> slide-l: ~a"
                     above-coords contact-coords slide-l)
      ;; Solve IK
      (send *baxter* :rotate-gripper :larm 0 :relative nil)
      (send *baxter* :slide-gripper :larm 0 :relative nil)
      ;; Move gripper above paper
      (send *ri* :angle-vector-raw (send *baxter* :rotate-wrist-ik :larm above-coords)
            :fast (send *ri* :get-arm-controller :larm) 0 :scale 3.0)
      (send *ri* :wait-interpolation)
      (send *baxter* :slide-gripper :larm (setq gripper-l 30) :relative nil)
      (send *ri* :angle-vector-raw (send *baxter* :larm :inverse-kinematics contact-coords)
            :fast (send *ri* :get-arm-controller :larm) 0 :scale 6.0)
      (send *ri* :wait-interpolation)
      ;; Slide paper
      (send *ri* :angle-vector-raw
            (send *baxter* :larm :move-end-pos (float-vector (- slide-l) 0 0) :local)
            :fast (send *ri* :get-arm-controller :larm) 0 :scale 30.0)
      (send *ri* :wait-interpolation)
      ;; Move gripper to grasp position
      (send *ri* :angle-vector-raw (send *baxter* :slide-gripper :larm 0 :relative nil)
            1000 (send *ri* :get-arm-controller :larm))
      (send *ri* :wait-interpolation)
      (setq avs
            (list (send *baxter* :larm :move-end-pos
                        (float-vector (- 10 target-protrusion) 0 0) :local)
                  (send *baxter* :larm :move-end-pos (float-vector 0 0 (- 3 gripper-l)) :local)))
      (send *ri* :angle-vector-sequence-raw avs
            :fast (send *ri* :get-arm-controller :larm) 0 :scale 20.0)
      (send *ri* :wait-interpolation)))
  (:grasp-paper ()
    (ros::ros-info "[:grasp-paper] Grasping paper...")
    (let ((roll-jt-b (aref (send *ri* :get-real-finger-av :larm) 2)) tm-to-grasp l-dist r-dist
                                                                     paper-p)
      (when (< roll-jt-b 110.0)
        (ros::ros-info "[:grasp-paper] roll-jt-b is ~a < 110, so initialize finger" roll-jt-b)
        ;; Turn off flex reflex
        (send *ri* :enable-flex-reflex :larm nil)
        (send *ri* :reset-i-prox-calib :larm)
        (send *ri* :move-hand :larm
              (send *baxter* :hand :larm :angle-vector #f(77 90 110)) 2000)
        (setq roll-jt-b 110.0))
      (setq tm-to-grasp (* 3000 (/ (- 180.0 roll-jt-b) 70)))  ;; Default value is 3000
      (ros::ros-info "[:grasp-paper] tm-to-grasp: ~a" tm-to-grasp)
      ;; Close finger until sensing paper
      ;; 160 is too far and 2000 is too fast to cancel.
      ;; After wait-interpolation, sensor values drop under threshold.
      ;; (send *ri* :move-hand :larm
      ;;       (send *baxter* :hand :larm :angle-vector #f(77 90 160)) 2000 :wait nil)
      (setq paper-p nil)
      (handler-bind ((fingers-touch-paper
                       #'(lambda (c)
                           (handler-bind ((fingers-touch-paper #'(lambda (c) nil)))
                             (ros::ros-info "[:grasp-paper] dist: l: ~a r: ~a"
                                            (send c :l-dist) (send c :r-dist))
                             (ros::ros-info "[:grasp-paper] Cancel :move-hand")
                             (send *ri* :cancel-move-hand :larm)
                             (setq paper-p t)))))
        (send *ri* :move-hand :larm
              (send *baxter* :hand :larm :angle-vector #f(77 90 180)) tm-to-grasp :wait t))
      ;; For logging
      (setq l-dist (send *ri* :get-i-prox-distance :larm :left-tip)
            r-dist (send *ri* :get-i-prox-distance :larm :right-tip))
      (ros::ros-info "[:grasp-paper] Final dist (only for log): l: ~a r: ~a" l-dist r-dist)
      (if paper-p
        (progn
          (ros::ros-info "[:grasp-paper] Paper is grasped")
          t)
        (progn
          (ros::ros-error "[:grasp-paper] Nothing is grasped")
          nil))))
  (:return-from-grasp-paper ()
    (ros::ros-info "[:return-from-grasp-paper] Returning from :grasp-paper...")
    (send *ri* :move-hand :larm
          (send *baxter* :hand-grasp-pre-pose :larm :cylindrical) 1000 :wait t)
    (send *ri* :angle-vector-raw
          (send *baxter* :larm :move-end-pos #f(0 0 200) :local)
          :fast (send *ri* :get-arm-controller :larm) 0 :scale 6.0)
    (send *ri* :wait-interpolation)
    ;; Turn on flex reflex
    (send *ri* :enable-flex-reflex :larm t)
    ;; Turn on all proximities
    (send *ri* :turn-off-proximity :intensity :larm nil)
    (send *ri* :turn-off-proximity :tof :larm nil))
  (:file-paper ()
    (ros::ros-info "[:file-paper] Filing paper...")
    (install-handler
      fingers-detach-paper
      #'(lambda (c)
          (handler-bind ((fingers-detach-paper #'(lambda (c) nil)))
            (ros::ros-info "[:file-paper] Cancel :larm angle vector")
            (send *ri* :cancel-angle-vector
                  :controller-type (send *ri* :get-arm-controller :larm))
            (ros::ros-error "[:file-paper] Paper is pulled out")
            (while
              (progn
                ;; Slightly open finger
                (send *ri* :move-hand :larm
                      (send *baxter* :hand :larm :angle-vector #f(77 90 140)) 1000)
                (ros::ros-info "[:file-paper] Waiting for paper to be inserted...")
                (handler-case (while t (unix::usleep (* 50 1000)))
                  (fingers-see-paper () (progn
                                          (ros::ros-info "[:file-paper] Paper is inserted")
                                          (not (send self :grasp-paper)))))))
            (send *ri* :resume-angle-vector))))
    (ros::ros-info "[:file-paper] Now you can pull out paper")
    (send *ri* :angle-vector-raw
          (send *baxter* :larm :move-end-pos #f(0 0 150) :local)
          :fast (send *ri* :get-arm-controller :larm) 0 :scale 6.0)
    (send *ri* :wait-interpolation)
    (assert (send *baxter* :larm :move-end-pos #f(450 0 0) :local))
    (send *ri* :angle-vector-raw (send *baxter* :angle-vector)
          :fast (send *ri* :get-arm-controller :larm) 0 :scale 7.0)
    (send *ri* :wait-interpolation)
    (remove-handler fingers-detach-paper)
    (ros::ros-info "[:file-paper] Now you cannot pull out paper")
    (let (avs)
      (setq avs (list (send *baxter* :larm :move-end-pos #f(0 0 -95) :local)
                      (send *baxter* :larm :move-end-pos #f(50 0 0) :local)))
      (send *ri* :angle-vector-sequence-raw avs
            :fast (send *ri* :get-arm-controller :larm) 0 :scale 20.0)
      (send *ri* :wait-interpolation)))
  (:return-from-file-paper ()
    (ros::ros-info "[:return-from-file-paper] Returning from :file-paper...")
    (send *ri* :move-hand :larm
          (send *baxter* :hand :larm :angle-vector #f(77 90 110)) 1000 :wait nil)
    (send *ri* :angle-vector-raw (send *baxter* :larm :move-end-rot 10 :y :local)
          :fast (send *ri* :get-arm-controller :larm) 0 :scale 4.0)
    (send *ri* :wait-interpolation)
    (send *ri* :move-hand :larm
          (send *baxter* :hand-grasp-pre-pose :larm :cylindrical) 1000)
    (send *baxter* :larm :move-end-rot -10 :y :local)
    (send *ri* :angle-vector-raw
          (send *baxter* :larm :move-end-pos #f(0 0 200) :local)
          :fast (send *ri* :get-arm-controller :larm) 0 :scale 10.0)
    (send *ri* :wait-interpolation)
    ;; Turn on flex reflex
    (send *ri* :enable-flex-reflex :larm t)
    ;; Turn on all proximities
    (send *ri* :turn-off-proximity :intensity :larm nil)
    (send *ri* :turn-off-proximity :tof :larm nil))
  (:mainloop-step ()
    (send *ri* :move-hand :larm
          (send *baxter* :hand-grasp-pre-pose :larm :opposed) 1000 :wait nil)
    (send self :move-larm->recognition-pose)
    (send *ri* :wait-interpolation)
    (send self :recognize-paper)
    (send self :slide-paper)
    (if (send self :grasp-paper)
      (progn
        (send self :file-paper)
        (send self :return-from-file-paper)
        t)
      (progn
        (send self :return-from-grasp-paper)
        nil)))
  (:mainloop ()
    (send self :recognize-table-box)
    (while (ros::ok)
      (send self :mainloop-step)
      (ros::spin-once))))

(defun jsk_arc2017_baxter::paper-filing-init (&key (ctype :default-controller) (moveit nil))
  (let (mvit-env mvit-rb)
    (when moveit
      (setq mvit-env (instance jsk_arc2017_baxter::baxterlgv8-moveit-environment))
      (setq mvit-rb (instance jsk_arc2017_baxter::baxterlgv8-robot :init)))
    (unless (boundp '*ri*)
      (setq *ri* (instance jsk_arc2017_baxter::baxterlgv8-interface :init :type ctype
                           :moveit-environment mvit-env
                           :moveit-robot mvit-rb))
      (load "package://roseus_resume/euslisp/interruption-handler.l")
      (roseus_resume:install-interruption-handler *ri*)
      (roseus_resume:register-callable-intervention
        (:intensity-prox-for-intervention (msg) sphand_driver_msgs::IntensityProxCalibInfoArray
          (let (l-dist r-dist)
            (setq l-dist (send *ri* :get-i-prox-distance :larm :left-tip))
            (setq r-dist (send *ri* :get-i-prox-distance :larm :right-tip))
            (when (and (< l-dist 70) (< r-dist 70))
              (signals fingers-see-paper :l-dist l-dist :r-dist r-dist))
            (when (and (< l-dist 25) (< r-dist 25))
              (signals fingers-touch-paper :l-dist l-dist :r-dist r-dist))
            (when (or (>= l-dist 35) (>= r-dist 35))
              (signals fingers-detach-paper :l-dist l-dist :r-dist r-dist))))))
    (unless (boundp '*baxter*)
      (setq *baxter* (instance jsk_arc2017_baxter::baxterlgv8-robot :init)))
    (unless (boundp '*co*)
      (setq *co* (when moveit (instance collision-object-publisher :init))))
    (unless (boundp '*ti*)
      (setq *ti* (instance jsk_arc2017_baxter::paper-filing-interface :init :moveit moveit)))
    (send *baxter* :angle-vector (send *ri* :state :potentio-vector))))