(defclass cube-object
  :super cascaded-link
  :slots (handle-top)
  )

(defmethod cube-object
  (:init
   (x y z &key (fillet 0))
   (send-super* :init nil)
    (let (cube-body cube-obj)
      (if (> fillet 0)
          (setq cube-body (make-cylinder-sector x y z fillet))
        (setq cube-body (make-cube x y z)))
      (send cube-body :set-color (float-vector 0.8 0.2 0.2 0.5))
      (send cube-body :translate (float-vector 0 0 (* 0.5 z)))
      (setq cube-obj (instance bodyset-link :init (make-cascoords) :bodies (list cube-body) :weight 0.2))
      (send self :assoc cube-obj)
      (send self :put :object-coords
            (make-cascoords :parent cube-obj :coords (send self :copy-worldcoords)))
      (setq handle-top (make-cascoords :coords (send cube-obj :copy-worldcoords) :name "handle-top"))
      (send handle-top :replace-pos (float-vector -35 -18 z))
      (send cube-obj :assoc handle-top)
      (setq links (list cube-obj))
      (send self :init-ending)
      )
    self
    )

  (:handle-top (&rest args) (forward-message-to handle-top args))
  )

(defun make-cylinder-sector (x y height radius &rest args
				&key (segments 12) (circumscribed nil)
				&allow-other-keys)
   (setq radius (float radius) height (float height))
   (let* ((theta (/ (/ pi 2.0) segments))
	      (radius (if circumscribed (/ radius (cos (/ theta 2.0))) radius))
	      (p (float-vector radius 0 0))
	      (base0 (list p))
          (base1 nil)
	      cyl)
     (dotimes (i (1- segments))
	   (push (rotate-vector (car base0) theta :z) base0))
     (dotimes (i segments)
	   (push (v+ (rotate-vector (elt (reverse base0) i) (* pi 0.5) :z) (float-vector (- (* radius 2) x) 0 0)) base1))
     (dotimes (i segments)
	   (push (v+ (rotate-vector (elt (reverse base0) i) pi :z) (float-vector (- (* radius 2) x) (- (* radius 2) y) 0)) base1))
     (dotimes (i segments)
	   (push (v+ (rotate-vector (elt (reverse base0) i) (* pi 1.5) :z) (float-vector 0 (- (* radius 2) y) 0)) base1))

     (setq base (append base1 base0))
      (setq cyl
	    (apply #'make-prism base height
		  :primitive  (list ':cylinder radius height segments)
		  args))
      (setf (get cyl :center-axis) (make-line (float-vector 0 0 0)
					                          (float-vector 0 0 height)))
      (send cyl :translate (float-vector (- (* x 0.5) radius) (- (* y 0.5) radius) (/ height -2.0)))
      cyl))
