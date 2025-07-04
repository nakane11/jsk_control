(require "cube-object.l")

(defclass smartphone
  :super cascaded-link
  :slots (handle-front handle-back)
  )

(defmethod smartphone
  (:init
   ()
   (send-super* :init nil)
   (let ((cube-body (make-cylinder-sector 69 144 8 8))
         (cut-screen (make-cylinder-sector 64 139 9 5.5))
         (screen (make-cylinder-sector 64 139 7.5 5.5))
         cube-obj)
     (send cut-screen :translate (float-vector 0 0 1))
     (send screen :set-color (float-vector 0.1 0.1 0.1 0.8))
     (send screen :translate (float-vector 0 0 0.5))
     (setq cube-body (body- cube-body cut-screen))
     (send cube-body :set-color (float-vector 0.8 0.5 0.5 1.0))
     (send cube-body :assoc screen)
     (send cube-body :translate (float-vector 0 0 (* 0.5 8)))
     (setq cube-obj (instance bodyset-link :init (make-cascoords) :bodies (list cube-body screen) :weight 0.2))
     (send self :assoc cube-obj)
     (setq handle-front (make-cascoords :coords (send (send cube-obj :copy-worldcoords) :translate (float-vector 0 60 8)) :name "handle-front"))
     (setq handle-back (make-cascoords :coords (send (send cube-obj :copy-worldcoords) :translate (float-vector 0 60 0)) :name "handle-back"))
     (send self :put :object-coords (make-cascoords :parent cube-obj :coords (send (send cube-obj :copy-worldcoords) :translate (float-vector 0 -72 0)) :name "object-coords")
           )
     (send cube-obj :assoc handle-front)
     (send cube-obj :assoc handle-back)
     (setq links (list cube-obj))
     (send self :init-ending)
     self
     ))

  (:handle-front (&rest args) (forward-message-to handle-front args))

  (:handle-back (&rest args) (forward-message-to handle-back args))
  )
