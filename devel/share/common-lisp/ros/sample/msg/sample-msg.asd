
(cl:in-package :asdf)

(defsystem "sample-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "sample_message" :depends-on ("_package_sample_message"))
    (:file "_package_sample_message" :depends-on ("_package"))
  ))