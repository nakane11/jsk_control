(require "models/arrow-object.l")

(defun make-scaled-arrow (&key (pos (float-vector 0 0 0)) (axis (float-vector 0 1 0))
                               (length 200.0)
                               (color (float-vector 1 0 0)))
  (let* ((ar (instance arrow-object :init))
         (size-scale (/ length 200.0))
         (bodies (send (car (send ar :links)) :bodies))
         (faces-list (send-all bodies :faces))
         (vertices-list (mapcar #'(lambda (x) (send-all x :vertices)) faces-list))
         scaled-vertices-list
         scaled-faces-list
         scaled-bodies
         scaled-arrow-link
         scaled-arrow
         )
    (setq scaled-vertices-list
          (mapcar #'(lambda (x) (mapcar #'(lambda (y) (mapcar #'(lambda (z) (scale size-scale z)) y)) x))
                  vertices-list))
    (setq scaled-faces-list
          (mapcar #'(lambda (x) (mapcar #'(lambda (y) (instance face :init :vertices y)) x))
                  scaled-vertices-list))
    (setq scaled-bodies
          (mapcar #'(lambda (x) (instance faceset :init :faces x)) scaled-faces-list))
    (send (elt scaled-bodies 1) :set-color :red)
    (send (elt scaled-bodies 2) :set-color :red)
    (send (elt scaled-bodies 3) :set-color :green)
    (send (elt scaled-bodies 4) :set-color :green)
    (send (elt scaled-bodies 5) :set-color :blue)
    (send (elt scaled-bodies 6) :set-color :blue)    
    (setq scaled-arrow-link
          (instance bodyset-link :init (make-cascoords)
                    :bodies scaled-bodies
                    :name :scaled-arrow-bodyset2
                    :weight 1
                    :centroid (float-vector 0.0 0.0 0.0)
                    :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
    (setq scaled-arrow (instance cascaded-link :init))
    (dolist (b scaled-bodies)
      (send scaled-arrow :assoc b))
    (send scaled-arrow :assoc scaled-arrow-link)
    (setq (scaled-arrow . links) (list scaled-arrow-link))
    (setq (scaled-arrow . joint-list) nil)
    (send scaled-arrow :init-ending)
    scaled-arrow
    ))
