
(cl:in-package :asdf)

(defsystem "tictoc_profiler-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ProfilerEntry" :depends-on ("_package_ProfilerEntry"))
    (:file "_package_ProfilerEntry" :depends-on ("_package"))
  ))