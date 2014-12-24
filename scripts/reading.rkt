#lang racket

(define-values (in out) (open-input-output-file "\\\\.\\com3"))

(define (close)
  (close-input-port in)
  (close-output-port out))

(define (read-ulong)
  (integer-bytes->integer (read-bytes 4 in) #f #t))

(define (read-int)
  (integer-bytes->integer (read-bytes 2 in) #f #t))

(define (print-reading)
  (let ([t (read-ulong)]
        [v (read-int)])
    (printf "~a, ~a\n" t v)))