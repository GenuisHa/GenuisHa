
(cl:in-package :asdf)

(defsystem "ucar_commander-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "my_nav_start" :depends-on ("_package_my_nav_start"))
    (:file "_package_my_nav_start" :depends-on ("_package"))
    (:file "nav_start" :depends-on ("_package_nav_start"))
    (:file "_package_nav_start" :depends-on ("_package"))
  ))