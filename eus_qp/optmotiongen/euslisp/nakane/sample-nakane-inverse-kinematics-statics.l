(require "nakane-hand.l")
(require "make-scaled-arrow.l")
(require "cube-object.l")
(require "util.l")
(require "../sample/sample-samplerobot.l")

(defun sample-hand-sqp-instant-manip-config-task
    (&key
     (optimize-torque? nil)
     (avoid-collision? t)
     (fix-obj? nil)
     (loop-num 100)
     )

  (setq *robot* (instance nakane-hand-robot :init))
  (setq *obj* (instance cube-object :init))

  ;; setup environment
  (setq *ground-butt2* (make-cube 200 200 20))
  (send *ground-butt2* :set-color (float-vector 0.5 0.5 0.25))
  (send *ground-butt2* :newcoords
        (make-coords :pos (float-vector -154 0 -10)))
  (setq *arrow-butt* (make-scaled-arrow :length 30))
  (send *arrow-butt* :newcoords
        (send (make-coords :pos (float-vector -154 37 0)) :copy-worldcoords))
  (setq *arrow-obj* (make-scaled-arrow :length 30))
  (send *arrow-obj* :newcoords
        (send (make-coords :pos (float-vector 0 0 0)) :copy-worldcoords))
  (setq *arrow-index* (make-scaled-arrow :length 30))
  (send *arrow-index* :newcoords (send *obj* :handle-top))
  (setq *env-list* (list *ground-butt2* *arrow-butt* *arrow-obj* *arrow-index*))
  (send-all *env-list* :worldcoords)

  ;; setup contact
  (let ((face- (elt (send *ground-butt2* :faces) 1)))
    (send face- :put :normal-tolerance-y 45.0) ;; [deg]
    (setq butt2-contact (generate-contact-ik-arg-from-rect-face :rect-face face-)))
  (setq butt-contact (instance contact-ik-arg :init
                                :target-coords (send *arrow-butt* :copy-worldcoords)
                                :translation-axis t
                                :rotation-axis nil
                                :transform-coords (send *arrow-butt* :copy-worldcoords)
                                ))
  (setq index-contact (instance contact-ik-arg :init
                                :target-coords (send *obj* :handle-top)
                                :translation-axis t
                                :rotation-axis nil
                                :transform-coords (send *obj* :handle-top)
                                ))
  (setq *contact-list*
        (list butt2-contact butt-contact index-contact))
  (setq *robot-obj-env*
        (instance robot-object-environment :init :robot *robot* :object *obj* :contact-list *contact-list*))

  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot* *obj*) *env-list*))

  ;; kin-target-coords: おしり-面，おしり-点，指-物体，物体-目標位置
  (setq *instant-manip-config-task*
        (instance* instant-manipulation-configuration-task :init
                   :robot-env *robot-obj-env*
                   :drive-joint-list
                   (if optimize-torque? (send *robot-obj-env* :drive-joint-list) nil)
                   :posture-joint-list
                   (send *robot-obj-env* :robot :joint-list)
                   :posture-joint-angle-list
                   (send-all (send *robot-obj-env* :robot :joint-list) :joint-angle)
                   :kin-target-coords-list
                   (remove
                    nil
                    (list (send butt2-contact :contact-coords)
                          (send butt-contact :contact-coords)
                          (send index-contact :contact-coords)
                          (send *arrow-obj* :copy-worldcoords)
                          ))
                   :kin-attention-coords-list
                   (remove
                    nil
                    (list (send *robot-obj-env* :robot :get :butt-coords2)
                          (send *robot-obj-env* :robot :get :butt-coords)
                          (send *robot-obj-env* :robot :get :index-fingertip-coords)
                          (send *robot-obj-env* :object :get :object-coords)
                          ))
                   :contact-target-coords-list
                    (list (send butt2-contact :contact-coords)
                          (send butt-contact :contact-coords)
                          (send index-contact :contact-coords)
                          (send *arrow-obj* :copy-worldcoords)
                         )
                   :contact-attention-coords-list
                    (list (send *robot-obj-env* :robot :get :butt-coords)
                          (send *robot-obj-env* :robot :get :butt-coords2)
                          (send *robot-obj-env* :robot :get :index-fingertip-coords)
                          (send *robot-obj-env* :object :get :object-coords)
                         )
                   :contact-constraint-list
                   (list
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 10 10 10))
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 10 10 10))
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 10 10 10))
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 10 10 10))
                    )
                   :contact-target-coords-obj-list
                   (list (send *robot-obj-env* :object :handle-top)
                         )
                   :contact-constraint-obj-list
                   (list
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 10 10 10))
                    )
                   :act-react-pair-list
                   (list
                    ;; (list (send *robot-obj-env* :object :handle-side-rarm) (send *robot-obj-env* :object :handle-side-rarm))
                    ;; (list (send *robot-obj-env* :object :handle-side-larm) (send *robot-obj-env* :object :handle-side-larm))
                    )
                   ;; :collision-pair-list
                   ;; (if avoid-collision?
                   ;;     (mapcar
                   ;;      #'(lambda (l)
                   ;;          (list l (send *robot-obj-env* :object :get :convex-hull-body)))
                   ;;      (flatten
                   ;;       (mapcar #'(lambda (limb) (send *robot-obj-env* :robot limb :links))
                   ;;               (list :torso :rleg :lleg))
                   ;;       )))
                   :collision-distance-margin 0.05
                   :delta-linear-joint 100 ;; [mm]
                   :delta-rotational-joint 30 ;; [deg]
                   (if optimize-torque?
                       (list
                        :norm-regular-scale-max 1e-6
                        ))
                   ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-manip-config-task*
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        ;; (validate-element-jacobian-manipulation :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num loop-num)
  )
