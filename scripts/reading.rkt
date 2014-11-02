#lang racket

(require racket/match)
(require racket/gui)

(define port-name "/dev/ttyUSB0")

(define in null)
(define out null)

(define (init-port)
  ; I have no idea what I'm doing here
  (system (string-append "stty -F " port-name " cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts"))
  (let-values ([(in-port out-port)
		(open-input-output-file port-name #:mode 'binary #:exists 'append)])
    (set! in in-port) 
    (set! out out-port)
    (file-stream-buffer-mode out 'none)))

(define (close)
  (close-input-port in)
  (flush-output out)
  (close-output-port out))

(define (read-ulong)
  (integer-bytes->integer (read-bytes 4 in) #f #t))

(define (read-int)
  (integer-bytes->integer (read-bytes 2 in) #f #t))

(define (read-float)
  (floating-point-bytes->real (read-bytes 4 in)))

(struct message (gyro))

(define (read-message)
  (let ([x (read-float)]
	[y (read-float)]
	[z (read-float)])
    (message (vector x y z))))

; a stream? well, sure, why not
(define (messages) (stream-cons (read-message) (messages)))

(define frame (new frame%
		   [label "Test"]
		   [width 400]
		   [height 400]))
(define canvas (new canvas%
		    [parent frame]))

(define (draw-readings msg)
  (define (vector-comp n v)
    (string-append n ": " (real->decimal-string v)))
  (define (draw dc)
    (send dc set-scale 2 2)
    (send dc set-text-foreground "blue")
    (for ([i '(0 1 2)]
	  [n '("x" "y" "z")]
	  [v (message-gyro msg)])
      (send dc draw-text (vector-comp n v) 0 (* 20 i))))
  (send canvas refresh-now draw))

(define (go)
  (init-port)
  (stream-for-each draw-readings (messages)))
