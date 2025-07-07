
(cl:in-package :asdf)

(defsystem "serial_comms-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Distances" :depends-on ("_package_Distances"))
    (:file "_package_Distances" :depends-on ("_package"))
  ))