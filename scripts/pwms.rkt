#lang racket
(require 2htdp/batch-io)

(define left-pwm 
  (read-csv-file/rows 
    "c:/stuff/rowboat/fixed_encoders_forward_left.csv"
    (lambda (row)
      [list (string->number (first row)) 
            (string->number (second row))])))