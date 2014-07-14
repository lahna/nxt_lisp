;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Institute for Artificial Intelligence/University
;;;       of Bremen nor the names of its contributors may be used to endorse or
;;;       promote products derived from this software without specific prior
;;;       written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :nxt-lisp)

(defvar *joint-command-publisher* nil)

;;; Startup. Sets subscribers
(defun startup ()
  (roslisp-utilities:startup-ros)
  (setf *joint-command-publisher*
        (roslisp:advertise "/joint_command"
                           "nxt_msgs/JointCommand"))
  (roslisp:subscribe "/touch" "nxt_msgs/Contact" #'touch-cb)
  (roslisp:subscribe "/ultrasonic" "sensor_msgs/Range" #'sensor-cb)
  (roslisp:subscribe "/color_sensor" "nxt_msgs/Color" #'color-cb)
  (roslisp:subscribe "/joint_states" "sensor_msgs/JointState" #'motorSens-cb)
  nil)
;;;----------------------------------------------------------------------------------
;;;defines Variable for motor State Sensor
(defvar *motorSens* nil)

;;;define callback for motor State Sensor
(defun motorSens-cb (msg)
  (roslisp:with-fields (position) msg
    (setf *motorSens* position)))

;;;define turn head (ultrasonic sensor)
(defmacro turn-head-until (condition)
 `(while ,condition)
  (set-motor-effort "motor_A" 0.7)
  `(loop while ,condition)
  (set-motor-effort "motor_A" 0.0))

(defmacro check-head-position-left ()
  `(< -1.3 (motorSens)))

(defmacro check-head-position-right ()
  `(> 1.3 (motorSens)))

;;;----------------------------------------------------------------------------------
;;; Function to set effort for one specific motor
(defun set-motor-effort (name effort)
  (roslisp:publish
   *joint-command-publisher*
   (make-message "nxt_msgs/JointCommand"
                 :name name
                 :effort effort)))

;;; Function so set effort for two explicid specified motors
(defun set-both-motors (effort)
  (set-motor-effort "motor_L" effort)
  (set-motor-effort "motor_R" effort))

;;; Stops both motors
(defun stop-both-motors ()
  (set-both-motors 0.0))

;;; var for touch-subscriber
(defvar *touch* nil)

;;; Defines callback for touch-subscriber
(defun touch-cb (msg)
  (roslisp:with-fields (contact) msg
    (setf *touch* contact)))

;;; returns if touched
(defun touch ()
  *touch*)

;;; var for sensor-subscriber
(defvar *sensor* nil)

;;; Defines callback for sensor-subscriber
(defun sensor-cb (msg)
  (roslisp:with-fields (range) msg
    (setf *sensor* range)))

;;; returns range until next object
(defun sensor ()
  *sensor*)

;;; sets both motors and stops if there is a wall in sight
(defun drive-until-wall ()
  (drive-until (check-range)))

;;; var for color-subscriber
(defvar *color* nil)

;;; color-callback
(defun color-cb (msg)
  (roslisp:with-fields (r) msg
    (setf *color* r)))

;;; returns true if there is something red
(defun color ()
  *color*)

;;; drives until there is something red
(defun drive-until-red ()
  (drive-until (check-color)))

;;; checks if somethings red
(defmacro check-color ()
  `(not(= 1.0 (color))))

;;; checks if sth is in range
(defmacro check-range ()
  `(< 0.5 (sensor)))

;;; Drives until a certain condition is false
(defmacro drive-until (condition)
  `(when ,condition
     (set-both-motors 0.7)
     (loop while ,condition)
     (stop-both-motors)))

;;; drives backwards until ""
(defmacro drive-backwards-until (condition)
  `(when ,condition
     (set-both-motors -0.7)
     (loop while ,condition)
     (stop-both-motors)))

;;; drives through the room until something is found
(defun drive-and-search-naiv ()
    (drive-until (and (check-color) (check-range)))
    (when (check-color)
      (set-motor-effort "motor_L" 0.7)
      (loop while (not (or (check-color) (check-range))))
      (stop-both-motors)
      (drive-and-search-naiv)))

;;; do a specific action for a specific timespan
(defmacro do-for-time (timespan start end)
  `(let ((time (get-universal-time)))
    (progn ,start)
    (loop while (< (- (get-universal-time) time) ,timespan))
    (progn ,end)))

(defmacro do-for-time-until (timespan start end check)
  `(let ((time (get-universal-time)))
     (progn ,start)
     (loop while (and (,check) (< (- (get-universal-time) time) ,timespan)))
     (progn ,end)))

;;; FIXME
(defun drive-and-search-backtracking ()
  (drive-until (and (check-color) (check-range)))
  (when (check-color)
    (do-for-time 0.5 (set-motor-effort "motor_L" 0.7) (stop-both-motors))
    (when (not (check-range))
      (drive-and-search-backtracking)
      (do-for-time 0.5 (set-motor-effort "motor_L" -0.7) (stop-both-motors)))))

;;; Drives backwards until wall is touched
(defun drive-backwards-until-wall ()
  (drive-backwards-until (not (touch))))
 
(defvar *gradunality* 1)

(defun gradunality ()
  *gradunality*)

(defvar *searched-list* (make-list 0))

(defun searched-list()
  *searched-list*)

(defun init-search-list ()
  (setf *searched-list* (make-list 0))
  (loop for i from 1 to (gradunality) do 
    (setf *searched-list*
          (append (searched-list) (make-list (gradunality) :initial-element 0)))))

;;; List: 0 = nothing known. 1 = explored 2 = wall 3 = red ball found

(defun main ()
  (loop for i from 0 to 1000 do
    (when T
      (print i)
      (drive-until-found i))))

(defun drive-until-found (i)
  (when (check-color)
    (if (check-range)
        (if (and (> i 0) (= 0 (mod i (gradunality))))
            (if (= 0 (mod (/ i (gradunality)) 2))
                (turn-right)
                (turn-left))
            (look-next-field))
        (turn-right))))

(defun turn-right ()
  (do-step "motor_L")
  (look-next-field)
  (do-step "motor_L"))

(defun turn-left ()
  (do-step "motor_R")
  (look-next-field)
  (do-step "motor_R"))

(defun look-next-field ()
    (do-for-time-until 2 (set-both-motors 0.7) (stop-both-motors) check-color))

(defun do-step (motor)
  (do-for-time-until 2 (set-motor-effort motor 0.75) (stop-both-motors) check-color))


;;;----------------------------------------------------------------------------------
;;; motor head movement
