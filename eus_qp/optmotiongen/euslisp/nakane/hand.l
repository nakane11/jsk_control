(defclass finger
  :super cascaded-link
  :slots (finger-name)
  )

(defmethod finger
  (:init
    (name)
    (let* ((b1 (make-cylinder 10 40))
           (b2 (make-cylinder 10 30))
           (b3 (make-cylinder 10 30))
           link joint)
      (send-super :init)
      (setq bodies (list b1 b2 b3))
      (setq finger-name name)
      (send b1 :set-color :red)
      (send b2 :set-color :green)
      (send b3 :set-color :blue)
      (dotimes (i (length bodies))
        (setq link (instance bodyset-link :init (make-cascoords)
                             :bodies (list (elt bodies i))
                             :name (format nil "~A_link~A" finger-name i)
                             ))
        (setq links (append links (list link)))
        )
      (send (elt links 1) :translate (float-vector 0 0 (elt ((send b1 :get :center-axis) . nvert) 2)))
      (send (elt links 2) :translate (float-vector 0 0 (+ (elt ((send b1 :get :center-axis) . nvert) 2)
                                                          (elt ((send b2 :get :center-axis) . nvert) 2))))
      (dotimes (i (1- (length links)))
        (send (elt links i) :assoc (elt links (+ i 1)))
        (setq joint (instance rotational-joint :init
                              :name (format nil "~A_joint~A" finger-name i)
                              :parent-link (elt links i) :child-link (elt links (+ i 1))
                              :axis :y
                              :min 0 :max 90
                              :max-joint-velocity 10.0 :max-joint-torque 999999.0
                              ))
        (setq joint-list (append joint-list (list joint)))
        )
      (send self :assoc (elt links 0))
      (setq end-coords (make-cascoords
                        :coords (send (elt links 2) :copy-worldcoords)
                        :name (format nil ":~A_endcoords" finger-name)))
      (send end-coords :translate (float-vector 0 0 (elt ((send b3 :get :center-axis) . nvert) 2)))
      (send (elt links 2) :assoc end-coords)
      (send self :put :end-coords end-coords)
      (send self :init-ending)
      ))
  )

;; (defclass hand
;;   :super cascaded-link
;;   :slots ()
;;   )

;; (defmethod hand
;;   (:init
;;     ()
;;     (let* ((b0 (make-cylinder 50 25))
;;            (b1 (make-cylinder 10 40))
;;            (b2 (make-cylinder 10 30))
;;            (b3 (make-cylinder 10 25))
;;            (bodies (list b0 b1 b2 b3))
;;            links)

;;       (send-all bodies :set-color (float-vector 0.8 0.8 0.8 1))
;;       (setq links (mapcar #'(lambda(b) (instance bodyset-link :init (make-cascoords) :bodies (list b))) bodies))
;;       (send (elt links 1) :assoc (elt links 2))
;;       (send (elt links 2) :assoc (elt links 3))

;;       )
;;     (send self :init-ending)
;;     )
;;   )
