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
    (send face- :put :margin 1) ;; [mm]
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
                    (list (send *robot-obj-env* :robot :get :butt-coords2)
                          (send *robot-obj-env* :robot :get :butt-coords)
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
(warn "(sample-hand-sqp-instant-manip-config-task)~%")

(defun sample-hand-sqp-trajectory-instant-manip-config-task-test
    (&key
     (optimize-torque? nil)
     (loop-num 50)
     (step 5)
     )

  (setq *robot* (instance nakane-hand-robot :init))
  (setq *obj* (instance cube-object :init))

  ;; setup environment
  (setq *ground-butt2-list* (mapcar #'(lambda (x) (make-cube 200 200 20)) (make-list step)))
  (dotimes (i (length *ground-butt2-list*))
    (let ((new-coords (send (send (make-coords :pos (float-vector -144 0 -10)) :copy-worldcoords) :translate (float-vector (* i 30) 0 0)))
          (col (float-vector 0.2 (- 0.7 (* i 0.05)) (+ 0.2 (* i 0.05)) 0.5)))
      (send (elt *ground-butt2-list* i) :set-color col)
      (send (elt *ground-butt2-list* i) :newcoords new-coords)
      ))

  (setq *arrow-butt-list* (mapcar #'(lambda (x) (make-scaled-arrow :length 30)) (make-list step)))
  (dotimes (i (length *arrow-butt-list*))
    (let ((new-coords (send (send (make-coords :pos (float-vector -144 37 0)) :copy-worldcoords) :translate (float-vector (* i 30) 0 0))))
      (send (elt *arrow-butt-list* i) :newcoords new-coords)
      ))

  (setq *arrow-obj-list* (mapcar #'(lambda (x) (make-scaled-arrow :length 30)) (make-list step)))
  (dotimes (i (length *arrow-obj-list*))
    (let ((new-coords (send (send (make-coords :pos (float-vector 0 0 0)) :copy-worldcoords) :translate (float-vector (* i 30) 0 0))))
      (send (elt *arrow-obj-list* i) :newcoords new-coords)
      ))

  (setq *arrow-index-list* (mapcar #'(lambda (x) (make-scaled-arrow :length 30)) (make-list step)))
  (dotimes (i (length *arrow-index-list*))
    (let ((new-coords (send (send (elt *arrow-obj-list* i) :copy-worldcoords) :translate (float-vector -35 -18 40))))
      (send (elt *arrow-index-list* i) :newcoords new-coords)
      ))

  (setq *env-list* (append *ground-butt2-list* *arrow-butt-list* *arrow-obj-list* *arrow-index-list*))
  (send-all *env-list* :worldcoords)

  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot* *obj*) *env-list*))

  (setq *robot-env-list* nil)
  (setq *instant-config-task-list* nil)

  (dotimes (i step)
      ;; setup contact
    (let ((face- (elt (send (elt *ground-butt2-list* i) :faces) 1)))
      (send face- :put :normal-tolerance-y 65.0) ;; [deg]
      (send face- :put :normal-tolerance-x 10.0) ;; [deg]
      (setq butt2-contact (generate-contact-ik-arg-from-rect-face :rect-face face-)))
    (setq butt-contact (instance contact-ik-arg :init
                                 :target-coords (send (elt *arrow-butt-list* i) :copy-worldcoords)
                                 :translation-axis t
                                 :rotation-axis nil
                                 :transform-coords (send (elt *arrow-butt-list* i) :copy-worldcoords)
                                 ))
    (setq index-contact (instance contact-ik-arg :init
                                 :target-coords (send (elt *arrow-index-list* i) :copy-worldcoords)
                                 :translation-axis t
                                 :rotation-axis nil
                                 :transform-coords (send (elt *arrow-index-list* i) :copy-worldcoords)
                                 ))
    (setq contact-list
          (list butt2-contact butt-contact index-contact))
    (setq robot-obj-env
          (instance robot-object-environment :init :robot *robot* :object *obj* :contact-list contact-list))

    (push (copy-object robot-obj-env) *robot-env-list*)

    (push
     (instance* instant-manipulation-configuration-task :init
                :robot-env (car *robot-env-list*)
                :drive-joint-list
                (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list) nil)
                :posture-joint-list
                (send (car *robot-env-list*) :robot :joint-list)
                :posture-joint-angle-list
                (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
                :kin-target-coords-list
                (remove
                 nil
                 (list ;; (send butt2-contact :contact-coords)
                       (send butt-contact :contact-coords)
                       (send index-contact :contact-coords)
                       (send (elt *arrow-obj-list* i) :copy-worldcoords)
                       ))
                :kin-attention-coords-list
                (remove
                 nil
                 (list ;; (send (car *robot-env-list*) :robot :get :butt-coords2)
                       (send (car *robot-env-list*) :robot :get :butt-coords)
                       (send (car *robot-env-list*) :robot :get :index-fingertip-coords)
                       (send (car *robot-env-list*) :object :get :object-coords)
                       ))
                :contact-target-coords-list
                (list (send butt2-contact :contact-coords)
                      (send butt-contact :contact-coords)
                      (send index-contact :contact-coords)
                      (send (elt *arrow-obj-list* i) :copy-worldcoords)
                      )
                :contact-attention-coords-list
                (list (send (car *robot-env-list*) :robot :get :butt-coords2)
                      (send (car *robot-env-list*) :robot :get :butt-coords)
                      (send (car *robot-env-list*) :robot :get :index-fingertip-coords)
                      (send (car *robot-env-list*) :object :get :object-coords)
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
                (list (send (car *robot-env-list*) :object :handle-top)
                      )
                :contact-constraint-obj-list
                (list
                 (instance 6d-min-max-contact-constraint
                           :init (float-vector 1000 1000 1000 10 10 10))
                 )
                :collision-distance-margin 0.05
                :delta-linear-joint 100 ;; [mm]
                :delta-rotational-joint 30 ;; [deg]
                (if optimize-torque?
                    (list
                     :norm-regular-scale-max 1e-6
                     ))
                )
     *instant-config-task-list*))

  (setq *trajectory-config-task*
        (instance trajectory-configuration-task :init
                  :instant-config-task-list (reverse *instant-config-task-list*)
                  :adjacent-regular-scale 1e-5
                  :torque-regular-scale 1e-3
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *trajectory-config-task*
                  ))
  (send *sqp-opt* :optimize :loop-num loop-num)

  (objects (append (list (car *robot-env-list*)) *env-list*))
  (send *trajectory-config-task* :play-animation
        :robot-env (car *robot-env-list*)
        :loop? nil
        :visualize-callback-func nil
        )

  )
(warn "(sample-hand-sqp-trajectory-instant-manip-config-task-test)~%")

(defun sample-hand-sqp-trajectory-instant-manip-config-task
    (&key
     (optimize-torque? nil)
     (loop-num 50)
     (step 4)
     )

  (setq *robot* (instance nakane-hand-robot :init))
  (setq *obj* (instance cube-object :init))


  ;; (setq *ground-butt2-list* (mapcar #'(lambda (x) (make-cube 200 200 20)) (make-list step)))
  (setq *arrow-butt-list* (mapcar #'(lambda (x) (make-scaled-arrow :length 30)) (make-list step)))
  (setq *arrow-obj-list* (mapcar #'(lambda (x) (make-scaled-arrow :length 30)) (make-list step)))
  (setq *arrow-index-list* (mapcar #'(lambda (x) (make-scaled-arrow :length 30)) (make-list step)))

  ;; (dotimes (i step)
  ;;   (let ((new-coords (send (send (make-coords :pos (float-vector -144 0 -10)) :copy-worldcoords) :translate (float-vector (* i 30) 0 0)))
  ;;         (col (float-vector 0.2 (- 0.7 (* i 0.05)) (+ 0.2 (* i 0.05)) 0.5)))
  ;;     (send (elt *ground-butt2-list* i) :set-color col)
  ;;     (send (elt *ground-butt2-list* i) :newcoords new-coords)
  ;;     ))

  (setq *robot-env-list* nil)
  (setq *instant-config-task-list* nil)

  (dotimes (i step)
    ;; setup environment
    (send (elt *arrow-butt-list* i) :newcoords (send (make-coords :pos (float-vector -144 37 0)) :translate (float-vector (* i 30) 0 0)))

    (send (elt *arrow-obj-list* i) :newcoords (send (make-coords :pos (float-vector 0 0 0)) :translate (float-vector (* i 30) 0 0)))
    (send (elt *arrow-index-list* i) :newcoords (send (send (elt *arrow-obj-list* i) :copy-worldcoords) :translate (send (send *obj* :handle-top) :copy-pos)))

    ;; setup contact
    (setq butt-contact (instance contact-ik-arg :init
                                 :name "butt-contact"
                                 :target-coords (send (elt *arrow-butt-list* i) :copy-worldcoords)
                                 :translation-axis t
                                 :rotation-axis nil
                                 :transform-coords (send (elt *arrow-butt-list* i) :copy-worldcoords)
                                 ))
    (setq index-contact (instance contact-ik-arg :init
                                  :name "index-contact"
                                  :target-coords (send (elt *arrow-index-list* i) :copy-worldcoords)
                                  :translation-axis t
                                  :rotation-axis nil
                                  :transform-coords (send (elt *arrow-index-list* i) :copy-worldcoords)
                                  ))
    (setq contact-list
          (list butt-contact index-contact))
    (setq *robot-obj-env*
          (instance robot-object-environment :init :robot *robot* :object *obj* :contact-list contact-list))

    (push (copy-object *robot-obj-env*) *robot-env-list*)
    (push
     (instance* instant-manipulation-configuration-task :init
                :robot-env (car *robot-env-list*)
                :drive-joint-list
                (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list) nil)
                :posture-joint-list
                (send (car *robot-env-list*) :robot :joint-list)
                :posture-joint-angle-list
                (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
                :kin-target-coords-list
                (remove
                 nil
                 (list ;; (send butt2-contact :contact-coords)
                  (send (send (car *robot-env-list*) :contact "butt-contact") :contact-coords)
                  (send (send (car *robot-env-list*) :contact "index-contact") :contact-coords)
                   (send (elt *arrow-obj-list* i) :copy-worldcoords)
                       ))
                :kin-attention-coords-list
                (remove
                 nil
                 (list ;; (send (car *robot-env-list*) :robot :get :butt-coords2)
                       (send (car *robot-env-list*) :robot :get :butt-coords)
                       (send (car *robot-env-list*) :robot :get :index-fingertip-coords)
                       (send (car *robot-env-list*) :object :get :object-coords)
                       ))
                :contact-target-coords-list
                (list ;; (send butt2-contact :contact-coords)
                      (send butt-contact :contact-coords)
                      (send index-contact :contact-coords)
                      (send (elt *arrow-obj-list* i) :copy-worldcoords)
                      )
                :contact-attention-coords-list
                (list ;; (send (car *robot-env-list*) :robot :get :butt-coords2)
                      (send (car *robot-env-list*) :robot :get :butt-coords)
                      (send (car *robot-env-list*) :robot :get :index-fingertip-coords)
                      (send (car *robot-env-list*) :object :get :object-coords)
                      )
                :contact-constraint-list
                (list
                 ;; (instance 6d-min-max-contact-constraint
                 ;;           :init (float-vector 1000 1000 1000 10 10 10))
                 (instance 6d-min-max-contact-constraint
                           :init (float-vector 1000 1000 1000 10 10 10))
                 (instance 6d-min-max-contact-constraint
                           :init (float-vector 1000 1000 1000 10 10 10))
                 (instance 6d-min-max-contact-constraint
                           :init (float-vector 1000 1000 1000 10 10 10))
                 )
                :contact-target-coords-obj-list
                (list (send (car *robot-env-list*) :object :handle-top)
                      )
                :contact-constraint-obj-list
                (list
                 (instance 6d-min-max-contact-constraint
                           :init (float-vector 1000 1000 1000 10 10 10))
                 )
                :collision-distance-margin 0.05
                :delta-linear-joint 100 ;; [mm]
                :delta-rotational-joint 30 ;; [deg]
                (if optimize-torque?
                    (list
                     :norm-regular-scale-max 1e-6
                     ))
                )
     *instant-config-task-list*)
    )

  (setq *env-list* (append  *arrow-butt-list* *arrow-obj-list* *arrow-index-list*))
  (send-all *env-list* :worldcoords)

  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot* *obj*) *env-list*))


  (setq *trajectory-config-task*
        (instance trajectory-configuration-task :init
                  :instant-config-task-list (reverse *instant-config-task-list*)
                  :adjacent-regular-scale 1e-5
                  :torque-regular-scale 1e-3
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *trajectory-config-task*
                  ))

  ;; play-animationの内部でrobot-envが書き換わってしまうためコピーを渡す
  (send *sqp-opt* :optimize :loop-num loop-num)
  (let ((robot-env (copy-object (car *robot-env-list*))))
    (objects (append (list robot-env) *env-list*))
    (send *trajectory-config-task* :play-animation
          :robot-env robot-env
          :loop? nil
          :visualize-callback-func nil
          ))
  *sqp-opt*
  )
(warn "(sample-hand-sqp-trajectory-instant-manip-config-task)~%")
