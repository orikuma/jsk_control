#!/usr/bin/env roseus

;; The most simplest version to execute footsteps by :set-foot-steps
;; method and have actionlib server interface
(ros::roseus "jaxon_footstep_controller" :anonymous nil)
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "nav_msgs")

(setq *step-refine* (ros::get-param "~/use_step_refine"))
(setq *modify-base-height* (ros::get-param "~/modify_base_height" nil))

(load "package://jsk_footstep_controller/euslisp/util.l")
;; (load "package://jsk_robot_utils/euslisp/robot-interface-utils.l")
(setq *robot-name* (ros::get-param "/robot/type" (unix::getenv "ROBOT")))
(load (robot-interface-file *robot-name*))
(init-robot-from-name *robot-name*)

(setq *plane-projection* (ros::get-param "~/plane_projection" nil))
(setq *planes* nil)
(setq *wait-for-plane* t)

;; (defun body-down (&key (robot *robot*) (z-pos 0) (crotch-pitch 0) (real t)) ;; 腰を下げる
;;   (send robot :reset-pose)
;;   (send robot :legs :move-end-pos (float-vector 0 0 z-pos) :local) ;;足が上がるから腰が下がる ;;#f(0 0 z-pos)はエラーをはく
;;   (send robot :legs :crotch-p :joint-angle crotch-pitch :relative t)
;;   (send robot :fix-leg-to-coords (make-coords)) ;makecoordsは何もないと0 0 0
;;   ;; setq *a* (make-coords :pos #f(100 100 100) :rpy (list 1 2 3))
;;   ;; send robot :fix-leg-to-coords *a*
;;   (send robot :move-centroid-on-foot :both '(:rleg :lleg))
;;   ;;(objects (list *robot*))
;;   (if real
;;       (send *ri* :angle-vector (send robot :angle-vector) 5000)
;;       (send *ri* :wait-interpolation)
;;       (unix:sleep 5)
;;     )
;;   )

(defun set-terrain-param-cb
  (req)
  (let ((res (send req :response)))
    ;; for toe step gait 
    (send *ri* :set-gait-generator-param
             :toe-angle 30 :heel-angle 5
             :toe-heel-phase-ratio #f(0.02 0.28 0.2 0.0 0.2 0.25 0.05)
             :stair-trajectory-way-point-offset #f(0.03 -0.0 0)
             :swing-trajectory-delay-time-offset 0.20
             :swing-trajectory-final-distance-weight 3.0
             :default-orbit-type :stair
             :default-step-time 1.5 ;; 1.5
             :default-double-support-ratio 0.2
             :default-step-height 0.05)

    ;; for kojio base link height modification
    ;; (send *ri* :stop-st)
    ;; (unix::sleep 3)
    
    ;; (send *ri* :set-st-param
    ;;     :use-limb-stretch-avoidance t ;; put base link down when the support leg is streching
    ;;     :limb-length-margin #f(0.14 0.14 0 0) ;; mergin from support leg stretching limit #f(rleg lleg rarm larm)
    ;;     :limb-stretch-avoidance-vlimit #f(-20e-5 1e-5) ;; velocity limit of base link for stretch avoidance
    ;;     )
    ;; (send *ri* :set-auto-balancer-param
    ;;     :use-limb-stretch-avoidance t ;; put base link down when the support leg is streching
    ;;     :limb-length-margin #f(0.12 0.12 0 0) ;; mergin from support leg stretching limit #f(rleg lleg rarm larm)
    ;;     )
    
    ;; (unix::sleep 3)
    ;; (send *ri* :start-st)
    res
    )
  )

(defun set-default-param-cb
  (req)
  (let ((res (send req :response)))
    ;; for toe step gait 
    (send *ri* :set-gait-generator-param
             :toe-angle 0.0 :heel-angle 0.0
             :toe-heel-phase-ratio #f(0.05 0.25 0.2 0.0 0.2 0.25 0.05)
             :stair-trajectory-way-point-offset #f(0.03 0.0 0.0)
             :swing-trajectory-delay-time-offset 0.15
             :swing-trajectory-final-distance-weight 3.0
             :default-orbit-type 4
             :default-step-time 1.2
             :default-double-support-ratio 0.35
             :default-step-height 0.065)
    ;; for kojio base link height modification
    ;; (send *ri* :stop-st)
    ;; (unix::sleep 3)
    
    ;; (send *ri* :set-st-param
    ;;     :use-limb-stretch-avoidance nil ;; put base link down when the support leg is streching
    ;;     :limb-length-margin #f(0.13 0.13 0.13 0.13) ;; mergin from support leg stretching limit #f(rleg lleg rarm larm)
    ;;     :limb-stretch-avoidance-vlimit #f(-0.0002 1.000000e-04) ;; velocity limit of base link for stretch avoidance
    ;;     )
    ;; (send *ri* :set-auto-balancer-param
    ;;     :use-limb-stretch-avoidance nil ;; put base link down when the support leg is streching
    ;;     ) 
   
    ;; (unix::sleep 3)
    ;; (send *ri* :start-st)
    res
    )
  )

(defun set-footstep-and-base-height-sequence (fsl)
  ;; (send *ri* :set-st-param
  ;;       :use-limb-stretch-avoidance t ;; 腰を脚長似あわせて下げるかどうか
  ;;       :limb-length-margin #f(0.145 0.145 0.145 0.145) ;; 伸びきりの脚長からどのくらいマージンを取るか #f(右足 左足 右腕 左腕)
  ;;       :limb-stretch-avoidance-vlimit #f(-20e-5 1e-5) ;; 腰変化の速度リミット
  ;;       )
  (let ((default-step-time (send (send *ri* :get-gait-generator-param) :default_step_time))
        avs tms)
    (setq avs (generate-base-pose-list fsl :default-pose (send *ri* :state :reference-vector)))
    (setq tms (make-list (length avs) :initial-element (* 1e3 default-step-time)))
  
  ;; (send *ri* :start-log)
    ;;(send *ri* :set-foot-steps *fsl*)
    ;; (send *ri* :set-foot-steps-with-base-height fsl avs tms)
    ;; (send *ri* :save-log "/tmp/dansa")

    (send *ri* :set-foot-steps-no-wait fsl)
    (send *ri* :angle-vector-sequence avs tms)

    ))

(defun generate-base-pose-list (footstep-coords &key (step-threshold 100.0)
                                             (robot *robot*)
                                             (default-pose (send robot :reset-pose)))
  (let ((step-list
         (mapcar #'(lambda (prev-fs fs)
                     (let ((cds (send prev-fs :transformation fs)))
                       (pprint cds)
                       (> (abs (elt (send cds :worldpos) 2)) step-threshold)))
                 (butlast footstep-coords) (cdr footstep-coords))) ;; list of "is next step is step up/down?" 
        pos len)
    (setq len (length step-list))
    ;; overwrite "t" in step-list to 0 and prev/next 2 steps to -2, -1, 1, 2 for sequential base-pose modification
    (while (setq pos (position t step-list))
      (when (>= (- pos 2) 0)
        (setf (elt step-list (- pos 2)) -2))
      (when (>= (- pos 1) 0)
        (setf (elt step-list (- pos 1)) -1))
      (setf (elt step-list pos) 0)
      (when (< (+ pos 1) len)
        (setf (elt step-list (+ pos 1)) 1))
      (when (< (+ pos 2) len)
        (setf (elt step-list (+ pos 2)) 2))
      )
    (pprint step-list)
    ;; overwrite step-list: each numbers -> appropriate modified base angle vectors, nil -> default-pose
    (mapcar #'(lambda (pos)
                (if pos
                    (let ()
                      (send robot :angle-vector default-pose)
                      (send robot :fix-leg-to-coords (make-coords))
                      (case pos ;; difference of base link height is constant
                        (-2
                         (send robot :legs :move-end-pos (float-vector 0 0 20)))
                        (-1
                         (send robot :legs :move-end-pos (float-vector 0 0 50)))
                        (0
                         (send robot :legs :move-end-pos (float-vector 0 0 80)))
                        (1
                         (send robot :legs :move-end-pos (float-vector 0 0 80)))
                        (2
                         (send robot :legs :move-end-pos (float-vector 0 0 50)))
                        )
                      (send robot :move-centroid-on-foot :both (list :rleg :lleg))
                      (send robot :angle-vector)
                      )
                  default-pose))
            step-list)
    ))

(defun separate-steps (footstep-coords &key (step-threshold 100.0)
                                            (robot *robot*)
                                            )
  (let ((step-list 
         (mapcar #'(lambda (prev-fs fs)
                     (let ((cds (send prev-fs :transformation fs)))
                       ;; (pprint cds)
                       (> (abs (elt (send cds :worldpos) 2)) step-threshold)))
                 (butlast footstep-coords) (cdr footstep-coords))) ;; list of "is next step is step up/down?" 
         (prev-pos 0) pos separated-footsteps)
    
    ;; separate footsteps by stepping 
    (while (setq pos (position t step-list))
      (when (> pos prev-pos)
        (push (subseq footstep-coords prev-pos (+ pos 1)) separated-footsteps)
        (setq prev-pos pos)
        (when (> (length footstep-coords) prev-pos)
          (push (subseq footstep-coords prev-pos (+ pos 3)) separated-footsteps) ;; assuming that left and right footsteps are in same plane least once (do not consider sequential steps)
          (setq prev-pos (+ pos 2)))
        )
      (setf (elt step-list pos) nil)
      )
    (when (subseq footstep-coords prev-pos (length footstep-coords))
      (push (subseq footstep-coords prev-pos (length footstep-coords)) separated-footsteps)
      )
    (reverse separated-footsteps)
    )
  )

(defun planes-cb (msg)
  (let ((polygons (send msg :polygons)))
    (let ((faces (mapcar #'(lambda (polygon)
                             (let (trans)
                               (send *tfl* :wait-for-transform (send polygon :header :frame_id) "map" (send polygon :header :stamp) 1)
                               (setq trans (send *tfl* :lookup-transform
                                                 (send polygon :header :frame_id)
                                                 "map"
                                                 (send polygon :header :stamp)))
                               (if trans
                                   (let ((points (mapcar #'(lambda (point)
                                                             (send (send (send trans :copy-worldcoords)
                                                                         :transform
                                                                         (make-coords :pos (ros::tf-point->pos point)))
                                                                   :worldpos))
                                                         (send polygon :polygon :points))))
                                     (instance face :init :vertices points))
                                 nil
                                 )
                               )
                             )
                         polygons)))
      (setq faces (remove-if #'null faces))
      (if faces
          (let ((fc (instance faceset :init :faces faces)))
            (setq *planes* faces)
            )))))

(defun walking-pose (original-av &key (real t) (height 40))
  (when (numberp height)
    ;; TODO: this line assumes that height of base_link does not changed from initial posture
    (send *robot* :angle-vector original-av)
    (send *robot* :legs :move-end-pos (float-vector 0 0 height)) 
    (when real
      ;; (send *ri* :set-gait-generator-param :default-orbit-type :stair)
      (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
      (send *ri* :wait-interpolation)
      )
    )
  )
#|
(setq *footsteps* nil)
(defun execute-cb (server goal) ;; old version
  (ros::ros-info "execute-cb")
  (let ((footstep (send goal :goal :footstep)))
    ;; footstep := jsk_recognition_msgs/FootstepArray
    (let ((footstep-coords (footstep-array->coords footstep))
          ;; (original-orbit-type (send (send *ri* :get-gait-generator-param) :default_orbit_type))
          )
      (print-readable-coords footstep-coords)
      ;;(walking-pose :real t :height 40)
      (send *ri* :set-foot-steps footstep-coords)
      (setq *footsteps* footstep-coords)

      (send *ri* :set-foot-steps-no-wait footstep-coords)
      (pprint footstep-coords)

      (let (ret map-dest cds-diff wait-until)
        (setq map-dest (copy-object (car (last footstep-coords))))
        (while
            (progn
              (setq ret (send *ri* :get-remaining-foot-step-sequence-current-index))
              (car ret))
          (when *step-refine*
            ;;
            (setq cds-diff (step-error-on-map (car ret) map-dest))

            (when (and (> (length (car ret)) 6) ;; offset + 4
                       (or (not wait-until)
                           (>= (cadr ret) wait-until)))
              (setq ret (send *ri* :get-remaining-foot-step-sequence-current-index))
              (refine-steps ret :offset 2 :expand-step cds-diff :exec t)
              (setq wait-until (+ (cadr ret) 2 1))
              )
            ) ;; step-refine
          (unix::usleep (* 1000 1000)) ;; TODO: use rostime
          ))
      ;;(send *ri* :angle-vector (send *robot* :reset-manip-pose) 3000)
      ;;(send *ri* :wait-interpolation)
      ;;(send *ri* :set-gait-generator-param :default-orbit-type original-orbit-type)
      (send server :set-succeeded (send server :result))
      )))
|#

;; (defun execute-cb2 (server goal)
;;   ;; first set-foot-steps
;;   (when (null *current-steps*)
;;     (let ((footstep-coords (footstep-array->coords (send goal :goal :footstep))))
;;       (when *debug*
;;         (pprint (list 'write *cntr*))
;;         (dump-structure (format nil "/tmp/footstepmsg~2,2D.l" *cntr*) goal)
;;         (incf *cntr*))
;;       (when *plane-projection*
;;         (when *wait-for-plane*
;;           (ros::ros-warn "plane is not subscribed, wait for planes")
;;           (while (not *planes*)
;;             (ros::spin-once)
;;             (ros::sleep)
;;             )
;;           )
;;         (setq footstep-coords (project-footsteps *planes* footstep-coords))
;;         )
;;       (pprint "new steps:")
;;       (print-readable-coords footstep-coords)
;;       ;; (setq *original-av* (send *ri* :state :potentio-vector))
;;       (if *modify-base-height*
;;           (set-footstep-and-base-height-sequence footstep-coords) ;; base link height should be set only once at first :set-foot-steps
;;         (send *ri* :set-foot-steps-no-wait footstep-coords)
;;         )
;;       (setq *current-steps* footstep-coords))
;;     (ros::ros-info "execute-cb: receive new steps ~D" (length *current-steps*))
;;     (return-from execute-cb2))

;;   (when *debug*
;;     (pprint (send goal :header :stamp))
;;     (print-ros-msg (send goal :goal_id))
;;     (terpri))

;;   (when (and (server . ros::pending-goal)
;;              (= (server . ros::status) actionlib_msgs::GoalStatus::*preempting*))
;;     ;; TODO: check resume
;;     ;; get new goal
;;     (let* ((new-goal (server . ros::pending-goal))
;;            (strategy (send new-goal :goal :strategy))
;;            (footstep-coords (footstep-array->coords (send new-goal :goal :footstep))))
;;       (cond
;;        ((= strategy jsk_footstep_msgs::execfootstepsgoal::*RESUME*) ;; resume (append)
;;         (when *debug*
;;           (pprint (list 'write *cntr*))
;;           (dump-structure (format nil "/tmp/footstepmsg~2,2D.l" *cntr*) new-goal)
;;           (incf *cntr*))
;;         ;; TODO: check old-goal last = new-goal first
;;         (setq *current-steps* (append-steps *current-steps* footstep-coords))

;;         (let* ((ret (send *ri* :get-remaining-foot-step-sequence-current-index))
;;                (idx (cadr ret))
;;                (abc-steps (butlast (car ret)))
;;                (abc-last-foot (car (last abc-steps)))
;;                abc-b-list
;;                )
;;           (unless (eq (send abc-last-foot :name)
;;                       (send (car footstep-coords) :name))
;;             (setq abc-last-foot (car (last (butlast abc-steps)))))
;;           (unless abc-last-foot
;;             (ros::ros-info "execute-cb: abc-last-foot is nil")
;;             (return-from execute-cb2)
;;             )
;;           (setq abc-b-list
;;                 (mapcar #'(lambda (x)
;;                             (let ((cds (send (send abc-last-foot :copy-worldcoords)
;;                                              :transform (send (car footstep-coords) :transformation x)
;;                                              )))
;;                               (send cds :name (send x :name))
;;                               cds))
;;                         footstep-coords))
;;           (setq abc-steps (append-steps abc-steps abc-b-list))
;;           (let ((sending-steps (subseq abc-steps *appending-offset*)))
;;             (when *plane-projection*
;;               (when *wait-for-plane*
;;                 (ros::ros-warn "plane is not subscribed, wait for planes")
;;                 (while (not *planes*)
;;                   (ros::spin-once)
;;                   (ros::sleep)
;;                   )
;;                 )
;;               (setq sending-steps (project-footsteps *planes* sending-steps))
;;               )
;;             (send *ri* :set-foot-steps-no-wait sending-steps
;;                   :overwrite-footstep-index (+ idx *appending-offset* 1)))
;;           (setq *wait-until* (+ idx *appending-offset* 1))
;;           )
;;         ;; dirty hack
;;         (setq (server . ros::pending-goal) nil)
;;         (setq (server . ros::status) actionlib_msgs::GoalStatus::*active*)
;;         (setq (server . ros::goal) new-goal)
;;         (setq (server . ros::goal-id) (send new-goal :goal_id))
;;         (ros::ros-info "execute-cb: append steps ~D / total ~D steps"
;;                        (length footstep-coords)
;;                        (length *current-steps*))
;;         (return-from execute-cb2)
;;         )
;;        ((= strategy jsk_footstep_msgs::execfootstepsgoal::*NEW_TARGET*) ;; new target (overwrite old one)
;;         ;;
;;         (return-from execute-cb2)
;;         )
;;        )))
;;   ;; check done
;;   (let ((ret (send *ri* :get-remaining-foot-step-sequence-current-index)))
;;     (unless (car ret)
;;       (setq *current-steps* nil)
;;       (setq *wait-until* nil)
;;       ;; (when *original-av*
;;       ;;   (send *ri* :angle-vector *original-av* 3000) ;; TODO: consider the case that execute footstep is not completed
;;       ;;   )
;;       (send server :set-succeeded (send server :result))
;;       (return-from execute-cb2)
;;       )
;;     (ros::ros-info "execute-cb: steps current ~D/~D (remaining ~D)"
;;                    (cadr ret) (length *current-steps*) (length (car ret)))
;;     )
;;   (when *step-refine*
;;     ;; step may be refined based on map
;;     (let (ret map-dest cds-diff wait-until)
;;       (setq map-dest (copy-object (car (last *current-steps*))))

;;       (setq ret (send *ri* :get-remaining-foot-step-sequence-current-index))
;;       (ros::spin-once)

;;       (setq cds-diff (calc-step-error-on-map (car ret) map-dest))

;;       (when (and (> (length (car ret)) 6) ;; offset + 4
;;                  (or (not *wait-until*)
;;                      (>= (cadr ret) *wait-until*)))
;;         (setq ret (send *ri* :get-remaining-foot-step-sequence-current-index))
;;         (pprint (list 'ret ret))
;;         (refine-steps ret :offset 1 :expand-step cds-diff :exec t :collision-avoid t)
;;         (setq *wait-until* (+ (cadr ret) 2 1))
;;         (ros::ros-info "execute-cb: step refine at ~D" *wait-until*)
;;         )
;;       ))
;;   )

(setq *footstep-buffers* nil)
(setq *current-steps* nil)
(setq *is-stepping* nil)
(setq *wait-until* nil)
(setq *appending-offset* 2)
(setq *original-av* nil)
;;(setq *cntr* 0)
(defun execute-cb3 (server goal)
  ;; first set-foot-steps
  (when (null *current-steps*)
    (let ((footstep-coords (footstep-array->coords (send goal :goal :footstep))))
      (when *debug*
        (pprint (list 'write *cntr*))
        (dump-structure (format nil "/tmp/footstepmsg~2,2D.l" *cntr*) goal)
        (incf *cntr*))
      (when *plane-projection*
        (when *wait-for-plane*
          (ros::ros-warn "plane is not subscribed, wait for planes")
          (while (not *planes*)
            (ros::spin-once)
            (ros::sleep)
            )
          )
        (setq footstep-coords (project-footsteps *planes* footstep-coords))
        )
      (pprint "new steps:")
      (print-readable-coords footstep-coords)

      ;; (send *robot* :angle-vector (send *ri* :state :reference-vector))
      ;; (send *robot* :legs :move-end-pos #f(0 0 50))
      ;; (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
      ;; (send *ri* :wait-interpolation)
      
      ;; (setq *original-av* (send *ri* :state :potentio-vector))
      
      ;; (if *modify-base-height*
      ;;     (set-footstep-and-base-height-sequence footstep-coords) ;; base link height should be set only once at first :set-foot-steps
      ;;   (send *ri* :set-foot-steps-no-wait footstep-coords)
      ;;   )
      ;; (setq *current-steps* footstep-coords)

      (pprint "separated steps:")
      (setq *footstep-buffers* (separate-steps footstep-coords))
      (pprint *footstep-buffers*)
      (setq *current-steps* (pop *footstep-buffers*))
      (setq *is-stepping* nil)
      (send *ri* :set-foot-steps-no-wait *current-steps*)
      
      )
    (ros::ros-info "execute-cb: receive new steps ~D" (length *current-steps*))
    (return-from execute-cb3))

  (when *debug*
    (pprint (send goal :header :stamp))
    (print-ros-msg (send goal :goal_id))
    (terpri))

  (when (and (server . ros::pending-goal)
             (= (server . ros::status) actionlib_msgs::GoalStatus::*preempting*))
    ;; TODO: check resume
    ;; get new goal
    (let* ((new-goal (server . ros::pending-goal))
           (strategy (send new-goal :goal :strategy))
           (footstep-coords (footstep-array->coords (send new-goal :goal :footstep))))
      (cond
       ((= strategy jsk_footstep_msgs::execfootstepsgoal::*RESUME*) ;; resume (append)
        (when *debug*
          (pprint (list 'write *cntr*))
          (dump-structure (format nil "/tmp/footstepmsg~2,2D.l" *cntr*) new-goal)
          (incf *cntr*))
        ;; TODO: check old-goal last = new-goal first
        (setq *current-steps* (append-steps *current-steps* footstep-coords))

        (let* ((ret (send *ri* :get-remaining-foot-step-sequence-current-index))
               (idx (cadr ret))
               (abc-steps (butlast (car ret)))
               (abc-last-foot (car (last abc-steps)))
               abc-b-list
               )
          (unless (eq (send abc-last-foot :name)
                      (send (car footstep-coords) :name))
            (setq abc-last-foot (car (last (butlast abc-steps)))))
          (unless abc-last-foot
            (ros::ros-info "execute-cb: abc-last-foot is nil")
            (return-from execute-cb3)
            )
          (setq abc-b-list
                (mapcar #'(lambda (x)
                            (let ((cds (send (send abc-last-foot :copy-worldcoords)
                                             :transform (send (car footstep-coords) :transformation x)
                                             )))
                              (send cds :name (send x :name))
                              cds))
                        footstep-coords))
          (setq abc-steps (append-steps abc-steps abc-b-list))
          (let ((sending-steps (subseq abc-steps *appending-offset*)))
            (when *plane-projection*
              (when *wait-for-plane*
                (ros::ros-warn "plane is not subscribed, wait for planes")
                (while (not *planes*)
                  (ros::spin-once)
                  (ros::sleep)
                  )
                )
              (setq sending-steps (project-footsteps *planes* sending-steps))
              )
            (send *ri* :set-foot-steps-no-wait sending-steps
                  :overwrite-footstep-index (+ idx *appending-offset* 1)))
          (setq *wait-until* (+ idx *appending-offset* 1))
          )
        ;; dirty hack
        (setq (server . ros::pending-goal) nil)
        (setq (server . ros::status) actionlib_msgs::GoalStatus::*active*)
        (setq (server . ros::goal) new-goal)
        (setq (server . ros::goal-id) (send new-goal :goal_id))
        (ros::ros-info "execute-cb: append steps ~D / total ~D steps"
                       (length footstep-coords)
                       (length *current-steps*))
        (return-from execute-cb3)
        )
       ((= strategy jsk_footstep_msgs::execfootstepsgoal::*NEW_TARGET*) ;; new target (overwrite old one)
        ;;
        (return-from execute-cb3)
        )
       )))
  ;; check done
  (let ((ret (send *ri* :get-remaining-foot-step-sequence-current-index)))
    (unless (car ret)
      (setq *current-steps* nil)
      (setq *wait-until* nil)
      ;; (when *original-av*
      ;;   (send *ri* :angle-vector *original-av* 3000) ;; TODO: consider the case that execute footstep is not completed
      ;;   )

      ;; (send *robot* :angle-vector (send *ri* :state :reference-vector))
      ;; (send *robot* :legs :move-end-pos #f(0 0 -50))
      ;; (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
      ;; (send *ri* :wait-interpolation)

      (let ((next-footstep-list (pop *footstep-buffers*)))
        (if next-footstep-list
            (progn
              (send *robot* :angle-vector (send *ri* :state :reference-vector))
              (if *is-stepping*
                  (progn
                    ;; base link up
                    (send *robot* :legs :move-end-pos #f(0 0 -80))
                    (setq *is-stepping* nil)
                    )
                (progn
                  ;; base link down
                  (send *robot* :legs :move-end-pos #f(0 0 80))
                  (setq *is-stepping* t)
                  )
                )
              (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
              (send *ri* :wait-interpolation)
              (setq *current-steps* next-footstep-list)
              (send *ri* :set-foot-steps-no-wait next-footstep-list)
              )
          (progn
            (when *is-stepping*
              (send *robot* :angle-vector (send *ri* :state :reference-vector))
              (send *robot* :legs :move-end-pos #f(0 0 -80))
              (setq *is-stepping* nil)
              (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
              (send *ri* :wait-interpolation)
              )
            (send server :set-succeeded (send server :result))
            )
          )
        )
      (return-from execute-cb3)
      )
    (ros::ros-info "execute-cb: steps current ~D/~D (remaining ~D)"
                   (cadr ret) (length *current-steps*) (length (car ret)))
    )
  (when *step-refine*
    ;; step may be refined based on map
    (let (ret map-dest cds-diff wait-until)
      (setq map-dest (copy-object (car (last *current-steps*))))

      (setq ret (send *ri* :get-remaining-foot-step-sequence-current-index))
      (ros::spin-once)

      (setq cds-diff (calc-step-error-on-map (car ret) map-dest))

      (when (and (> (length (car ret)) 6) ;; offset + 4
                 (or (not *wait-until*)
                     (>= (cadr ret) *wait-until*)))
        (setq ret (send *ri* :get-remaining-foot-step-sequence-current-index))
        (pprint (list 'ret ret))
        (refine-steps ret :offset 1 :expand-step cds-diff :exec t :collision-avoid t)
        (setq *wait-until* (+ (cadr ret) 2 1))
        (ros::ros-info "execute-cb: step refine at ~D" *wait-until*)
        )
      ))
  )

(ros::subscribe (format nil "~A/planes" (ros::get-name)) jsk_recognition_msgs::PolygonArray #'planes-cb)
(setq *server* (instance ros::simple-action-server :init
                         (ros::get-name)
                         jsk_footstep_msgs::ExecFootstepsAction
                         :groupname "footstep_action"
                         :execute-cb 'execute-cb3
                         ))

(ros::rate 2)
(ros::ros-info "simple footstep controller is ready as ~A" (ros::get-name))
(ros::advertise-service "/footstep_controller/set_terrain_param" std_srvs::Empty #'set-terrain-param-cb)
(ros::advertise-service "/footstep_controller/set_default_param" std_srvs::Empty #'set-default-param-cb)

(while (ros::ok)
  (send *server* :worker)
  (send *server* :spin-once)
  (ros::spin-once)
  (ros::sleep)
  )
