(require "nakane-hand.l")
(require "make-scaled-arrow.l")
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

(defun sample-hand-trajectory-ik-with-butt
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     &allow-other-keys
     )
  (setq *robot* (instance nakane-hand-robot :init))
  (let ((first-step 130) (stride 60))
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
                            (send *robot* :get :butt-coords)
                            (send *robot* :get :index-fingertip-coords))
         :rotation-axis-list
         (make-list 6 :initial-element (make-list 2 :initial-element nil))
         :debug-view (list :additional-draw-objects (append *arrow-right-list* *arrow-left-list*))
         :root-virtual-mode :6dof
         :root-virtual-joint-invariant? nil
         :pre-process-func pre-process-func
         :post-process-func post-process-func
         ))
  (objects (append (list *robot*) *arrow-right-list* *arrow-left-list*))
  (visualize-trajectory-result *ret* :visualize-callback-func visualize-callback-func)
  *ret*
  )
