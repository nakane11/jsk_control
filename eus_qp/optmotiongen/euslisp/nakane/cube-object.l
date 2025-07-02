(defclass cube-object
  :super cascaded-link
  :slots (handle-top)
  )

(defmethod cube-object
  (:init
   ()
   (send-super* :init nil)
    (let* ((cube-body (make-cube 80 80 60))
           cube-obj)
      (send cube-body :set-color (float-vector 0.8 0.2 0.2 0.5))
      (send cube-body :translate (float-vector 0 0 (* 0.5 (z-of-cube cube-body))))
      (setq cube-obj (instance bodyset-link :init (make-cascoords) :bodies (list cube-body) :weight 0.2))
      (send self :assoc cube-obj)
      (send self :put :object-coords
            (make-cascoords :parent cube-obj :coords (send self :copy-worldcoords)))
      (setq handle-top (make-cascoords :coords (send cube-obj :copy-worldcoords)))
      (send handle-top :replace-pos (float-vector -35 -18 (z-of-cube cube-body)))
      (send cube-obj :assoc handle-top)
      (setq links (list cube-obj))
      (send self :init-ending)
      )
    self
    )

  (:handle-top (&rest args) (forward-message-to handle-top args))
  )
