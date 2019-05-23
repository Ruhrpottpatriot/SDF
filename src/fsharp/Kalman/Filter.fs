namespace Kalman

open Sensor
open MathNet.Numerics.LinearAlgebra

module Filter =
    let predict F D ss cs =
        let state = F * (List.head ss)
        let transposed: Matrix<float> = F.Transpose()
        let f:Matrix<float> = F * ((List.head cs) * transposed) + D

        state::ss, f::cs
        
        // let state = F * List.head ss
        // let covar =(F * ((List.head cs) * F.Transpose()) + D
        // (state::ss), (covar::cs)

    let filter (H: Matrix<float>) (R: Matrix<float>) (ss: Coordinate list) (cs: Matrix<float> list)=, m =
        let v = m - (H * ss.Head)
        let S = (H * (cs.Head * H.Transpose())) + R
        let W = (cs.Head * (H.Transpose() * S.Invert()))

        let state = ss.Head + (W * v)
        let covar = cs.Head - (W * (S * W.Transpose()))
        (state::(ss.Tail)), (covar::(cs.Tail))