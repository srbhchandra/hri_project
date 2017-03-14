
(cl:in-package :asdf)

(defsystem "face_recognition-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "faces" :depends-on ("_package_faces"))
    (:file "_package_faces" :depends-on ("_package"))
  ))