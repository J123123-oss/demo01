
(cl:in-package :asdf)

(defsystem "serial_comms-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Distances" :depends-on ("_package_Distances"))
    (:file "_package_Distances" :depends-on ("_package"))
    (:file "INSPVAE" :depends-on ("_package_INSPVAE"))
    (:file "_package_INSPVAE" :depends-on ("_package"))
  ))