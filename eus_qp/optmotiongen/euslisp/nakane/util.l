(require "../sample/sample-inverse-kinematics.l")

(defun draw-sleep
    ()
  (unix::usleep 10000))

(defun draw-result
    ()
  (visualize-trajectory-result *ret* :visualize-callback-func #'draw-sleep))

(defun make-face
  (pos rot height width)
  (let ((board (make-cube width height 5)) face-)
    (send board :newcoords (send (make-coords :pos pos :rot rot) :copy-worldcoords))
    (send board :set-color (float-vector (+ (/ (random 20) 30.0) 0.3) (+ (/ (random 20) 30.0) 0.3) (+ (/ (random 20) 30.0) 0.3) 0.5))
    (send board :translate (float-vector 0 0 (* -0.5 (z-of-cube board))) :local)
    (setq face- (elt (send board :faces) 1))
    (send face- :put :margin 1) ;; [mm]
    (send face- :put :normal-move-target-axis :z)
    (send face- :put :normal-tolerance-y 45.0) ;; [deg]
    (send face- :put :normal-tolerance-x 5.0) ;; [deg]
    (cons face- board)
    ))
