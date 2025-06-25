(load "package://soft_exterior_hand/urdf/soft-exterior-hand.l")

(defclass nakane-hand-robot
  :super soft_exterior_hand-robot
  :slots ()
  )

(defmethod nakane-hand-robot
  (:init
   ()
   (send-super :init)
   (send self :add-fingertips)
   (send self :init-ending)
   )
  (:add-fingertips
   ()
   (let* ((index-link-3 (send self :index_link_3_lk))
          (index-dummy-body (make-cube 1e-3 1e-3 1e-3))
          (middle-link-3 (send self :middle_link_3_lk))
          (middle-dummy-body (make-cube 1e-3 1e-3 1e-3)))
     (setq index-fingertip-link (instance bodyset-link
                                           :init (make-cascoords)
                                           :bodies (list index-dummy-body)
                                           :name "index-fingertip"
                                           :weight 0))
     (send index-fingertip-link :newcoords (send index-link-3 :copy-worldcoords))
     (send index-link-3 :assoc index-fingertip-link)
     (setq index-fingertip-joint
           (instance rotational-joint :init
                     :name "index-fingertip-joint"
                     :parent-link index-link-3 :child-link index-fingertip-link
                     :axis (float-vector 0 0 1)
                     :min 0 :max 0
                     :max-joint-velocity 0
                     :max-joint-torque 0))
     (setq index-fingertip-coords (make-cascoords
                              :coords (send index-fingertip-link :copy-worldcoords)
                              :name ":index-fingertip-coords"))
     (send index-fingertip-link :assoc index-fingertip-coords)
     (send self :put :index-fingertip-coords index-fingertip-coords)
     (setq middle-fingertip-link (instance bodyset-link
                                           :init (make-cascoords)
                                           :bodies (list middle-dummy-body)
                                           :name "middle-fingertip"
                                           :weight 0))
     (send middle-fingertip-link :newcoords (send middle-link-3 :copy-worldcoords))     
     (send middle-link-3 :assoc middle-fingertip-link)
     (setq middle-fingertip-joint
           (instance rotational-joint :init
                     :name "middle-fingertip-joint"
                     :parent-link middle-link-3 :child-link middle-fingertip-link
                     :axis (float-vector 0 0 1)
                     :min 0 :max 0
                     :max-joint-velocity 0
                     :max-joint-torque 0))
     (setq middle-fingertip-coords (make-cascoords
                              :coords (send middle-fingertip-link :copy-worldcoords)
                              :name ":middle-fingertip-coords"))
     (send middle-fingertip-link :assoc middle-fingertip-coords)
     (send self :put :middle-fingertip-coords middle-fingertip-coords)     
     (setq links (append links (list index-fingertip-link middle-fingertip-link)))
     (setq joint-list (append joint-list (list index-fingertip-joint middle-fingertip-joint)))
   ))
  )
