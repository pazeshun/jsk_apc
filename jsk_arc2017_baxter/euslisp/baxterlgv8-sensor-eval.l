(require "package://jsk_arc2017_baxter/euslisp/lib/baxterlgv8-interface.l")

(defun instance-init ()
  (jsk_arc2017_baxter::baxterlgv8-init :type :larm-controller)
  (objects (list *baxter*)))

(defun turn-off-needless-sensors ()
  (send *ri* :turn-off-proximity :intensity :larm
        (list :left-root :right-root
              :palm-bottom :palm-top :palm-left))  ;; turn on at least 3 sensors to get value
  (send *ri* :turn-off-proximity :tof :larm
        (list :left-root :right-root
              :palm-bottom :palm-top :palm-left)))

(defun turn-on-all-sensors ()
  (send *ri* :turn-off-proximity :intensity :larm nil)
  (send *ri* :turn-off-proximity :tof :larm nil))

;; Evaluation of measured distance
(setq *dist-eval-av*
      #f(-67.6978 47.3957 -29.272 -83.5225 71.1338 60.2944 93.7255 -158.034 0.0 0.0
                  -81.3647 4.96582 -4.13086 79.8706 -137.791 -10.459 -67.5439 0.552233 0.357853))
(defun dist-eval-init ()
  (send *ri* :angle-vector (send *baxter* :angle-vector *dist-eval-av*) 5000)
  (send *ri* :move-hand :larm
        (send *baxter* :hand :larm :angle-vector (float-vector 90 45 45)) 1000)
  (send *ri* :wait-interpolation)
  (send *ri* :enable-flex-reflex :larm nil)
  (turn-off-needless-sensors))
(defun dist-eval ()
  (send *ri* :calib-proximity-threshold :larm)
  (send *ri* :reset-i-prox-calib :larm)
  (unix::sleep 1)
  ;; Enter calib zone
  (send *baxter* :larm :move-end-pos #f(0 0 -90) :world)
  (send *ri* :angle-vector (send *baxter* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  (let ((loop-num 7) (service-name "/synchronized_proximity_printer/start_printing"))
    (ros::wait-for-service service-name)
    (dotimes (i loop-num)
      (unix::sleep 3)
      (format t "Stable start time: ~a~%" (ros::time-now))
      (ros::service-call service-name (instance std_srvs::EmptyRequest :init))
      (unix::sleep 3)
      (format t "Stable end time: ~a~%" (ros::time-now))
      (when (< i (1- loop-num))
        (send *baxter* :larm :move-end-pos #f(0 0 -10) :world)
        (send *ri* :angle-vector (send *baxter* :angle-vector) 2000)
        (send *ri* :wait-interpolation)))))

;; Evaluation of picking paper
(setq *pick-paper-av*
      #f(-67.6978 47.3957 -29.272 -83.5225 71.1338 60.2944 93.7255 -158.034 0.0 0.0
                  -81.3647 4.96582 -4.13086 79.8706 -137.791 -10.459 -67.5439 0.552233 0.357853))
(defun pick-paper-init ()
  (send *ri* :angle-vector (send *baxter* :angle-vector *pick-paper-av*) 5000)
  (send *ri* :move-hand :larm
        (send *baxter* :hand-grasp-pre-pose :larm :opposed) 1000)
  (send *ri* :wait-interpolation)
  (send *ri* :enable-flex-reflex :larm nil)
  (turn-off-needless-sensors))
(defun pick-paper (&key (intensity nil))
  (send *ri* :calib-proximity-threshold :larm)
  (send *ri* :reset-i-prox-calib :larm)
  (send *ri* :angle-vector (send *baxter* :larm :move-end-pos #f(0 0 -90) :world) 3000)
  (send *ri* :wait-interpolation)
  (unix::sleep 1)
  (send *ri* :move-hand :larm
        (send *baxter* :hand :larm :angle-vector (float-vector 5 72 72)) 2000)
  (send *ri* :move-hand :larm
        (send *baxter* :hand :larm :angle-vector (float-vector 5 90 90)) 2000 :wait nil)
  (let (cancel-func paper-p l-prox r-prox l-thre r-thre)
    (if intensity
      (progn
        (setq l-thre 7000 r-thre 3500)
        (setq cancel-func
              #'(lambda ()
                  (setq paper-p
                        (and
                          (>= (setq l-prox (send *ri* :get-i-proximity :larm :left-tip))
                              l-thre)
                          (>= (setq r-prox (send *ri* :get-i-proximity :larm :right-tip))
                              r-thre))))))
      (progn
        (setq l-thre 25 r-thre 25)
        (setq cancel-func
              #'(lambda ()
                  (setq paper-p
                        (and
                          (< (setq l-prox (send *ri* :get-i-prox-distance :larm :left-tip))
                             l-thre)
                          (< (setq r-prox (send *ri* :get-i-prox-distance :larm :right-tip))
                             r-thre)))))))
    (send *ri* :wait-interpolation-until-funcs :larm (list cancel-func) :hand)
    (ros::ros-info "Final prox: l: ~a r: ~a" l-prox r-prox)
    (if paper-p
      (progn
        (ros::ros-info "Paper is grasped. Shift to lifting")
        (unix::sleep 1)
        (send *ri* :angle-vector
              (send *baxter* :larm :move-end-pos #f(0 0 90) :world) 2000)
        (send *ri* :wait-interpolation))
      (progn
        (ros::ros-error "Nothing is grasped")))))
