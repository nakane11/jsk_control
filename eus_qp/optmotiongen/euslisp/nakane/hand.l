(defclass finger
  :super cascaded-link
  :slots (finger-name)
  )

(defmethod finger
  (:init
    (name)
    (let* ((finger-name name)
           (dummy-link (instance bodyset-link
                                 :init (make-cascoords)
                                 :bodies (list (make-cube 1e-3 1e-3 1e-3))
                                 :name (format nil "~A_metacarpal_link" finger-name)
                                 :weight 0))
           (b1 (make-cylinder 10 40))
           (b2 (make-cylinder 10 30))
           (b3 (make-cylinder 10 30))
           (bodies (list b1 b2 b3))
           link joint)
      (send-super :init)
      (setq links (append links (list dummy-link)))
      ;; リンクの定義
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
      (send (elt links 2) :translate (float-vector 0 0 (elt ((send b1 :get :center-axis) . nvert) 2)))
      (send (elt links 3) :translate (float-vector 0 0 (+ (elt ((send b1 :get :center-axis) . nvert) 2)
                                                          (elt ((send b2 :get :center-axis) . nvert) 2))))
      ;; 外転関節
      (setq metacarpal-joint (instance rotational-joint :init
                                       :name (format nil "~A_metacarpal_joint" finger-name)
                                       :parent-link dummy-link :child-link (elt links 1)
                                       :axis :x
                                       :min -40 :max 40
                                       :max-joint-velocity 10.0 :max-joint-torque 999999.0
                                       ))
      (setq joint-list (append joint-list (list metacarpal-joint)))
      ;; 屈曲関節
      (dotimes (i 2)
        (send (elt links (+ i 1)) :assoc (elt links (+ i 2)))
        (setq joint (instance rotational-joint :init
                              :name (format nil "~A_joint~A" finger-name i)
                              :parent-link (elt links (+ i 1)) :child-link (elt links (+ i 2))
                              :axis :y
                              :min 0 :max 90
                              :max-joint-velocity 10.0 :max-joint-torque 999999.0
                              ))
        (setq joint-list (append joint-list (list joint)))
        )

      (send self :assoc dummy-link)
      (send dummy-link :assoc (elt links 1))
      (send dummy-link :rotate (deg2rad 90) :y)
      (setq end-coords (make-cascoords
                        :coords (send (elt links 2) :copy-worldcoords)
                        :name (format nil ":~A_endcoords" finger-name)))
      (send end-coords :translate (float-vector 0 0 (elt ((send b3 :get :center-axis) . nvert) 2)))
      (send (elt links 2) :assoc end-coords)
      (send self :put :end-coords end-coords)
      (send self :init-ending)
      self
      ))
  )

(defclass hand
  :super cascaded-link
  :slots ()
  )

(defmethod hand
  (:init
    ()
    (send-super :init)
    (let* ((b0 (make-cylinder 10 25))
           (finger-names (list "index" "middle" "ring" "little"))
           fingers)
      ;; 掌の定義
      (send b0 :set-color (float-vector 0.8 0.8 0.8 1))
      (send b0 :translate (float-vector 0 0 -12.5))
      (setq palm-link (instance bodyset-link :init (make-cascoords)
                                :bodies (list b0)
                                :name "palm_link"
                                ))
      (send self :assoc palm-link)
      (setq links (list palm-link))
      ;; 指の定義
      (dolist (finger-name finger-names)
        (let* ((dummy-link (instance bodyset-link
                                     :init (make-cascoords)
                                     :bodies (list (make-cube 1e-3 1e-3 1e-3))
                                     :name (format nil "dummy_~A_link" finger-name)
                                     :weight 0))
               (finger (instance finger :init finger-name))
               (dummy-joint-x (instance linear-joint :init
                                        :name (format nil "dummy_~A_x_joint" finger-name)
                                        :parent-link palm-link :child-link dummy-link
                                        :min -1000 :max 1000
                                        :axis (float-vector 1 0 0)))
               (dummy-joint-y (instance linear-joint :init
                                        :name (format nil "dummy_~A_y_joint" finger-name)
                                        :parent-link dummy-link
                                        :child-link (elt (send finger :links) 0)
                                        :min -1000 :max 1000
                                        :axis (float-vector 0 1 0))))
          ;; (send finger :rotate (deg2rad 90) :y)
          (send palm-link :assoc dummy-link)
          (print (elt (send finger :links) 0))
          (send dummy-link :assoc (elt (send finger :links) 0))
          (setq links (append links (list dummy-link) (flatten (send finger :links))))
          (setq joint-list (append joint-list (list dummy-joint-x dummy-joint-y) (flatten (send finger :joint-list))))))
      (send self :init-ending)
      self
      ))
  )
