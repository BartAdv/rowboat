open System.IO

let path = """C:\stuff\rowboat\fixed_encoders_forward_right.csv"""

File.ReadAllLines(path)
|> Seq.map (fun r -> r.Split([|',';';'|]))
|> Seq.map (fun vs -> (int vs.[0]), (float vs.[1]))
|> Seq.groupBy fst
|> Seq.map (fun (pwm, vs) -> pwm, (Seq.averageBy snd vs))
|> Seq.iter (fun (pwm, avg) -> printfn "%d; %O" pwm avg)