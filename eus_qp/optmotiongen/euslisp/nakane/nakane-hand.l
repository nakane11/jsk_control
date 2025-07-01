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
   (send self :add-butt)
   (send self :init-ending)
   )
  (:add-fingertips
   ()
   (let* ((index-link-3 (send self :index_link_3_lk))
          (middle-link-3 (send self :middle_link_3_lk))
          (ring-link-3 (send self :ring_link_3_lk)))
     (setq index-fingertip-coords (make-cascoords
                              :coords (send index-link-3 :copy-worldcoords)
                              :name ":index-fingertip-coords"))
     (send index-fingertip-coords :translate #f(13 0 2) :local)
     (send index-fingertip-coords :rotate (deg2rad -20) :y :local)
     (send index-link-3 :assoc index-fingertip-coords)
     (send self :put :index-fingertip-coords index-fingertip-coords)

     (setq middle-fingertip-coords (make-cascoords
                              :coords (send middle-link-3 :copy-worldcoords)
                              :name ":middle-fingertip-coords"))
     (send middle-fingertip-coords :translate #f(13 0 2) :local)
     (send middle-fingertip-coords :rotate (deg2rad -20) :y :local)
     (send middle-link-3 :assoc middle-fingertip-coords)
     (send self :put :middle-fingertip-coords middle-fingertip-coords)

     (setq ring-fingertip-coords (make-cascoords
                              :coords (send ring-link-3 :copy-worldcoords)
                              :name ":ring-fingertip-coords"))
     (send ring-fingertip-coords :translate #f(13 0 2) :local)
     (send ring-fingertip-coords :rotate (deg2rad -20) :y :local)
     (send ring-link-3 :assoc ring-fingertip-coords)
     (send self :put :ring-fingertip-coords ring-fingertip-coords)     
     ))

  (:add-butt
   ()
   (let* ((base-link (send self :base_link_lk)))
     (setq butt-link (instance bodyset-link
                               :init (make-cascoords)
                               :bodies (list (make-cube 1e-3 1e-3 1e-3))
                               :name "butt_link"
                               :weight 0))
     (send butt-link :newcoords (send base-link :copy-worldcoords))
     (send butt-link :translate #f(-13 0 0) :local)
     (send butt-link :rotate (deg2rad 45) :y :local)
     (send base-link :assoc butt-link)
     (setq butt-joint
           (instance rotational-joint :init
                     :name "butt-joint"
                     :parent-link base-link :child-link butt-link
                     :axis (float-vector 0 1 0)
                     :min 0 :max 0
                     :max-joint-velocity 0
                     :max-joint-torque 0))
     (setq butt-coords (make-cascoords
                        :coords (send butt-link :copy-worldcoords)
                        :name ":butt-coords"))
     (send butt-link :assoc butt-coords)
     (send self :put :butt-coords butt-coords)

     (setq butt-link2 (instance bodyset-link
                               :init (make-cascoords)
                               :bodies (list (make-cube 1e-3 1e-3 1e-3))
                               :name "butt_link2"
                               :weight 0))
     (send butt-link2 :newcoords (send base-link :copy-worldcoords))
     (send butt-link2 :translate #f(-13 -5 0) :local)
     (send butt-link2 :rotate (deg2rad 45) :y :local)
     (send base-link :assoc butt-link2)
     (setq butt-joint2
           (instance rotational-joint :init
                     :name "butt-joint2"
                     :parent-link base-link :child-link butt-link2
                     :axis (float-vector 0 1 0)
                     :min 0 :max 0
                     :max-joint-velocity 0
                     :max-joint-torque 0))
     (setq butt-coords2 (make-cascoords
                        :coords (send butt-link2 :copy-worldcoords)
                        :name ":butt-coords2"))
     (send butt-link2 :assoc butt-coords2)
     (send self :put :butt-coords2 butt-coords2)

     (setq links (append links (list butt-link butt-link2)))
     (setq joint-list (append joint-list (list butt-joint butt-joint2)))
     ))
  )
