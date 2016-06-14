#!/usr/bin/env roseus

(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")

(ros::load-ros-manifest "jsk_footstep_planner")
(ros::roseus "footstep_modify") ;; registering node as footstep_modify

(defun init ()
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
)

(defun foot-step-modify (&key (index 0) 
			 (fslist ())
			 (modify #f(0 0 0)))
  (let (newstep (make-coords :coords (send (nth (+ 1 index) fslist) :translate modify :world)))
    (if (eq (length fslist) (+ index 1)) ()  (setq fslist (append (subseq fslist 0 index)
								  (cons newstep (nthcdr (+ 2 index) fslist)))))
  )
)

(defun go-steady (&key (index 0)
		       (fslist ()) )
  (send *ri* :set-foot-steps-no-wait (list
  				      (nth index fslist) (nth (+ 1 index) fslist)))
  ;; This doesn't work!!
  ;; (send *ri* :set-foot-steps-no-wait fslist
  ;; 	:overwrite-footstep-index (+ 1 index))
)

;;registered as callback function
;;execute when footstep is subscribed
(defun fsgetter (msg)
  (setq *footsteplist* (list)) ;; empty list
  (let (fs_msgs footsteplist)
    (setq footsteplist (list))
    (setq fs_msgs (send msg :footsteps))
    (print "subscribe callbuck function is executed!")
    (print fs_msgs)
    (dolist (fs_msg fs_msgs)
      (let (fscoords fsleg fs)
	(setq fscoords (ros::tf-pose->coords (send fs_msg :pose)))
	(case (send fs_msg :leg)
	      (1 (setq fs (make-coords :coords fscoords :name :lleg)))
	      (2 (setq fs (make-coords :coords fscoords :name :rleg))))
	      ;;(jsk_footstep_msgs::Footstep::*LLEG* (setq fs (make-coords :coords fscoords :name :lleg)))
	      ;;(jsk_footstep_msgs::Footstep::*RLEG* (setq fs (make-coords :coords fscoords :name :rleg))))
	;; (print (send fs_msg :leg))
	;; (setq fs (make-coords :coords fscoords :name (send fsleg :name)))
	(print fs)
	(push fs footsteplist)
      )
    )
    (setq *footsteplist* (reverse footsteplist))
  )
  (dotimes (ind (- (length *footsteplist*) 1))
    (print ind)
    (foot-step-modify :index ind :fslist *footsteplist* :modify (float-vector 0 0 0))
    (go-steady :index ind :fslist *footsteplist*)
    (print *footsteplist*)
    (send *ri* :wait-foot-steps)
  )
)

(ros::subscribe "footstep_modify/footstep" jsk_footstep_msgs::FootstepArray #'fsgetter)
(init)

;; (send *Ri* :go-pos-no-wait 3 0 0)
;; (setq fsl (send *Ri* :get-remaining-foot-step-sequence))
;; (send *Ri* :emergency-walking-stop)
;; (setq reversed-fsl (reverse fsl))

(do-until-key
 (ros::spin-once)
)

;; (dotimes (ind (- (length fsl) 1))
;;   (print ind)
;;   (foot-step-modify :index ind :fslist reversed-fsl :modify (float-vector (* 100 ind) 0 0))
;;   (print reversed-fsl)
;;   (go-steady :index ind :fslist reversed-fsl)
;;   (send *ri* :wait-foot-steps)
;; )

