(require "nakane-hand.l")
(require "make-scaled-arrow.l")
(require "util.l")
(require "../sample/sample-inverse-kinematics.l")

(defun sample-hand-trajectory-ik-with-fingers
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     &allow-other-keys
     )
  (setq *robot* (instance nakane-hand-robot :init))
  (setq *arrow-right-list* (mapcar #'(lambda (x) (make-scaled-arrow :length 30)) (make-list 6)))
  (send (elt *arrow-right-list* 0) :newcoords (make-coords :pos #f(0 20 0)))
  (send (elt *arrow-right-list* 1) :newcoords (make-coords :pos #f(80 20 0)))
  (send (elt *arrow-right-list* 2) :newcoords (make-coords :pos #f(80 20 0)))
  (send (elt *arrow-right-list* 3) :newcoords (make-coords :pos #f(120 20 0)))
  (send (elt *arrow-right-list* 4) :newcoords (make-coords :pos #f(120 20 0)))
  (send (elt *arrow-right-list* 5) :newcoords (make-coords :pos #f(160 20 0)))

  (setq *arrow-left-list* (mapcar #'(lambda (x) (make-scaled-arrow :length 30)) (make-list 6)))
  (send (elt *arrow-left-list* 0) :newcoords (make-coords :pos #f(80 -20 0)))
  (send (elt *arrow-left-list* 1) :newcoords (make-coords :pos #f(80 -20 0)))
  (send (elt *arrow-left-list* 2) :newcoords (make-coords :pos #f(120 -20 0)))
  (send (elt *arrow-left-list* 3) :newcoords (make-coords :pos #f(120 -20 0)))
  (send (elt *arrow-left-list* 4) :newcoords (make-coords :pos #f(160 -20 0)))
  (send (elt *arrow-left-list* 5) :newcoords (make-coords :pos #f(160 -20 0)))

  (setq
   *ret*
   (send *robot*
         :inverse-kinematics-trajectory-optmotiongen
         (mapcar
          #'(lambda (rc lc) (list rc lc))
          (send-all *arrow-right-list* :copy-worldcoords)
          (send-all *arrow-left-list* :copy-worldcoords)
          )
         :move-target-list (list
                            (send *robot* :get :middle-fingertip-coords)
                            (send *robot* :get :index-fingertip-coords))
         :rotation-axis-list
         (make-list 6 :initial-element (make-list 2 :initial-element nil))
         :debug-view (list :additional-draw-objects (append *arrow-right-list* *arrow-left-list*))
         :root-virtual-mode :planar
         :root-virtual-joint-invariant? nil
         :pre-process-func pre-process-func
         :post-process-func post-process-func
         ))
  (objects (append (list *robot*) *arrow-right-list* *arrow-left-list*))
  (visualize-trajectory-result *ret* :visualize-callback-func visualize-callback-func)
  *ret*
  )

(defun sample-hand-trajectory-ik-with-butt-face
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     &allow-other-keys
     )
  (setq *robot* (instance nakane-hand-robot :init))

  (let ((first-step 110) (stride 60))
    (setq *arrow-right-list* (mapcar #'(lambda (x) (make-scaled-arrow :length 30)) (make-list 6)))
    (send (elt *arrow-right-list* 0) :newcoords (make-coords :pos (float-vector 0 0 0)))
    (send (elt *arrow-right-list* 1) :newcoords (make-coords :pos (float-vector stride 0 0)))
    (send (elt *arrow-right-list* 2) :newcoords (make-coords :pos (float-vector stride 0 0)))
    (send (elt *arrow-right-list* 3) :newcoords (make-coords :pos (float-vector (* stride 2) 0 0)))
    (send (elt *arrow-right-list* 4) :newcoords (make-coords :pos (float-vector (* stride 2) 0 0)))
    (send (elt *arrow-right-list* 5) :newcoords (make-coords :pos (float-vector (* stride 3) 0 0)))

    (setq *arrow-left-list* (mapcar #'(lambda (x) (make-scaled-arrow :length 30)) (make-list 6)))
    (send (elt *arrow-left-list* 0) :newcoords (make-coords :pos (float-vector first-step -60 0)))
    (send (elt *arrow-left-list* 1) :newcoords (make-coords :pos (float-vector first-step -60 0)))
    (send (elt *arrow-left-list* 2) :newcoords (make-coords :pos (float-vector (+ first-step stride) -60 0)))
    (send (elt *arrow-left-list* 3) :newcoords (make-coords :pos (float-vector (+ first-step stride) -60 0)))
    (send (elt *arrow-left-list* 4) :newcoords (make-coords :pos (float-vector (+ first-step (* stride 2)) -60 0)))
    (send (elt *arrow-left-list* 5) :newcoords (make-coords :pos (float-vector (+ first-step (* stride 2)) -60 0)))
    )

  (setq *face-list* nil)
  (setq *board-list* nil)
  (dotimes (i (length *arrow-right-list*))
    (let* ((pos- (send (elt *arrow-right-list* i) :pos))
           (rot- (send (elt *arrow-right-list* i) :rot))
           (tmp (make-face pos- rot- 500 500))
           (face- (car tmp)) (board- (cdr tmp)))
      (setq *face-list* (append *face-list* (list face-)))
      (setq *board-list* (append *board-list* (list board-)))))

  (setq
   *ret*
   (send *robot*
         :inverse-kinematics-trajectory-optmotiongen
         (mapcar
          #'(lambda (butt butt2 lc) (list butt butt2 lc))
          (send-all *arrow-right-list* :copy-worldcoords)
          *face-list*
          (send-all *arrow-left-list* :copy-worldcoords)
          )
         :move-target-list (list
                            (send *robot* :get :butt-coords)
                            (send *robot* :get :butt-coords2)
                            (send *robot* :get :index-fingertip-coords))
         :rotation-axis-list
         (make-list 6 :initial-element (list nil nil nil))
         :debug-view (list :additional-draw-objects (append *arrow-right-list* *arrow-left-list*))
         :root-virtual-mode :6dof
         :root-virtual-joint-invariant? nil
         :pre-process-func pre-process-func
         :post-process-func post-process-func
         ))
  (objects (append (list *robot*) *arrow-right-list* *arrow-left-list* *board-list*))
  *ret*
  )

(defun sample-carry-cube
    ()
  (unless (boundp '*irtviewer*)
    (make-irtviewer))
  (setq *robot* (instance nakane-hand-robot :init))
  (let ((cube- (make-cube 80 80 40)) finger-face (stride 40))
    (send cube- :translate (float-vector 250 20 (* 0.5 (z-of-cube cube-))))
    (setq finger-face (elt (send cube- :faces) 1))
    (send finger-face :put :margin 1)
    (send finger-face :put :normal-move-target-axis :z)
    (send finger-face :put :normal-tolerance 10.0)

    (let* ((butt-body (make-face (float-vector 105 20 0) #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0)) 140 200))
           (butt-face (car butt-body)) (butt-cube (cdr butt-body)))
      (objects (list *robot* cube- butt-cube))

      ;; 箱の上に人差し指と中指を置く
      (send *robot*
            :inverse-kinematics-optmotiongen
            (list butt-face finger-face finger-face)
            :move-target (list (send *robot* :get :butt-coords2)
                               (send *robot* :get :index-fingertip-coords)
                               (send *robot* :get :middle-fingertip-coords))
            :rotation-axis (list nil t t)
            :debug-view t
            :root-virtual-mode :6dof
            :root-virtual-joint-invariant? nil
            )

      (setq new-butt-coords (make-coords))
      (send new-butt-coords :replace-pos (send (send (send *robot* :get :butt-coords) :copy-worldcoords) :pos))
      (send new-butt-coords :translate (float-vector stride 0 0) :world)
      (setq butt-body (make-face (send new-butt-coords :pos) (send new-butt-coords :rot) 140 200))
      (setq butt-face (car butt-body))
      (setq butt-cube (cdr butt-body))
      (setq butt-arrow (make-scaled-arrow :length 30))
      (send butt-arrow :newcoords (send new-butt-coords :copy-worldcoords))

      (setq new-middle-coords (make-coords))
      (send new-middle-coords :replace-pos (send (send (send *robot* :get :middle-fingertip-coords) :copy-worldcoords) :pos))
      (setq middle-arrow (make-scaled-arrow :length 30))
      (send middle-arrow :newcoords new-middle-coords)

      (objects (list *robot* cube- butt-cube butt-arrow middle-arrow))
      (print new-middle-coords)
      (print new-butt-coords)

      (send *robot*
            :inverse-kinematics-optmotiongen
            (list new-middle-coords new-butt-coords)
            :move-target (list (send *robot* :get :middle-fingertip-coords)
                               (send *robot* :get :butt-coords))
            :rotation-axis (list nil nil)
            :debug-view t
            :root-virtual-mode :6dof
            )
      )
    )
  )

(defun sample-hand-trajectory-ik-cube
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     &allow-other-keys
     )
  (setq *robot* (instance nakane-hand-robot :init))

  (let ((cube- (make-cube 80 80 40))
        (butt-target (make-coords :pos #f(66.456 32.626 -0.774)))
        (index-target (make-coords :pos #f(211.049 -9.337 40.235)))
        (stride 40))
    (send cube- :translate (float-vector 250 20 (* 0.5 (z-of-cube cube-))))
    (objects (list *robot* cube-))
    ;; 箱の上に人差し指を置く
    (send *robot*
          :inverse-kinematics-optmotiongen
          (list butt-target index-target)
          :move-target (list (send *robot* :get :butt-coords2)
                             (send *robot* :get :index-fingertip-coords))
          :rotation-axis (list nil nil)
          :debug-view t
          :root-virtual-mode :6dof
          :root-virtual-joint-invariant? nil
          )

    (send (send *robot* :get :index-fingertip-coords) :assoc cube-)

    ;; (setq *index-list*
    ;;       (list (make-coords :pos (float-vector 211.049 -9.337 40.235))
    ;;             (make-coords :pos (float-vector (+ stride 211.049) -9.337 40.235))))

    ;; (setq *butt-list*
    ;;       (list (make-coords :pos (float-vector (+ stride 66.456) 32.626 -0.774))
    ;;             (make-coords :pos (float-vector (+ stride 66.456) 32.626 -0.774))))

    ;; (setq
    ;;  *ret*
    ;;  (send *robot*
    ;;        :inverse-kinematics-trajectory-optmotiongen
    ;;        (mapcar
    ;;         #'(lambda (butt index) (list butt index))
    ;;         (send-all *butt-list* :copy-worldcoords)
    ;;         (send-all *index-list* :copy-worldcoords)
    ;;         )
    ;;        :move-target-list (list (send *robot* :get :butt-coords)
    ;;                                (send *robot* :get :index-fingertip-coords))
    ;;        :rotation-axis-list
    ;;        (make-list 2 :initial-element (list nil nil))
    ;;        :debug-view t
    ;;        :root-virtual-mode :6dof
    ;;        :root-virtual-joint-invariant? nil
    ;;        ))
    ))
