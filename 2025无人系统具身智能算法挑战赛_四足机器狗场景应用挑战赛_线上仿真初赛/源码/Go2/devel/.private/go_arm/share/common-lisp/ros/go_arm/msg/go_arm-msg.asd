
(cl:in-package :asdf)

(defsystem "go_arm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Mycaryolo" :depends-on ("_package_Mycaryolo"))
    (:file "_package_Mycaryolo" :depends-on ("_package"))
  ))