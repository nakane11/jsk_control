(require "cube-object.l")

(defclass smartphone
  :super cube-object
  :slots (handle-front handle-back)
  )

(defmethod smartphone
  (:init
   ()
   (send-super :init 69 144 8)
   (let ((obj (car (send self :links))))
     (setq handle-front (make-cascoords :coords (send (send obj :copy-worldcoords) :translate (float-vector 0 60 8)) :name "handle-front"))
     (setq handle-back (make-cascoords :coords (send (send obj :copy-worldcoords) :translate (float-vector 0 60 0)) :name "handle-back"))
     (send self :put :object-coords (make-cascoords :parent obj :coords (send (send obj :copy-worldcoords) :translate (float-vector 0 -72 0)) :name "object-coords")
           )
     (send obj :assoc handle-front)
     (send obj :assoc handle-back))
   (send self :init-ending)
   self
   )

  (:handle-front (&rest args) (forward-message-to handle-front args))

  (:handle-back (&rest args) (forward-message-to handle-back args))
  )
