
(cl:in-package :asdf)

(defsystem "apriltags-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Stop" :depends-on ("_package_Stop"))
    (:file "_package_Stop" :depends-on ("_package"))
    (:file "Start" :depends-on ("_package_Start"))
    (:file "_package_Start" :depends-on ("_package"))
    (:file "RunningIds" :depends-on ("_package_RunningIds"))
    (:file "_package_RunningIds" :depends-on ("_package"))
    (:file "StopAll" :depends-on ("_package_StopAll"))
    (:file "_package_StopAll" :depends-on ("_package"))
    (:file "IsRunning" :depends-on ("_package_IsRunning"))
    (:file "_package_IsRunning" :depends-on ("_package"))
    (:file "IsIdOpen" :depends-on ("_package_IsIdOpen"))
    (:file "_package_IsIdOpen" :depends-on ("_package"))
  ))