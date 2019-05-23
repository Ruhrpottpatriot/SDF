namespace Kalman
open MathNet.Numerics.LinearAlgebra.Single
open MathNet.Numerics.LinearAlgebra.Complex32
module Sensor =   
    open MathNet.Numerics.Distributions
    open MathNet.Numerics.LinearAlgebra
    open FSharp.Data.UnitSystems.SI.UnitSymbols
    
    [<Measure>]
    type Radian

    type SphericalCoordinate = { Radius:float; Theta:float<Radian>; Phi:float<Radian> }
    type PolarCoordinate = { Radius: float; Theta:float<Radian> }
    type Carthesian2DCoordinate = { X:float; Y:float }      
    type Carthesian3DCoordinate = { X:float; Y:float; Z:float }
    
    type Coordinate = 
        | Spherical of SphericalCoordinate
        | Polar of PolarCoordinate
        | Carthesian2D of Carthesian2DCoordinate
        | Carthesian3D of Carthesian3DCoordinate

    let ConvertToSpherical (c: Carthesian3DCoordinate) =
        let r = sqrt (c.X**2.0 + c.Y ** 2.0 + c.Z **2.0)
        let theta = (atan2 c.Y c.X) * 1.0<Radian>
        let phi = acos (c.Z / r) * 1.0<Radian>
        { Radius = r; Theta = theta; Phi = phi }

    let ConvertToCarthesian3D (c: SphericalCoordinate) = 
        let x = c.Radius * cos (c.Theta / 1.0<Radian>) * sin (c.Phi / 1.0<Radian>)
        let y = c.Radius * sin (c.Theta / 1.0<Radian>) * cos (c.Theta / 1.0<Radian>)
        let z = c.Radius * cos (c.Theta / 1.0<Radian>)
        { X = x; Y = y; Z = z }

    let ConvertToPolar (c: Carthesian2DCoordinate) =
        let r = c.X**2.0 |> (+) <| c.Y**2.0 |> sqrt
        let theta = atan2 c.Y c.X |> (*) 1.0<Radian>
        { Radius = r; Theta = theta }

    let ConvertToCarthesian2D (c: PolarCoordinate) =
        let x = c.Radius * cos (c.Theta / 1.0<Radian>)
        let y = c.Radius * sin (c.Theta / 1.0<Radian>)
        { X = x; Y = y }

    let (*) (m:Matrix<float>) (c: Coordinate) : Coordinate =
        match c with
        | _ -> c

            
    type Carthesian2DCoordinate with
        static member Origin = {X = 0.0; Y = 0.0 }
        static member (+) (l:Carthesian2DCoordinate, r:Carthesian2DCoordinate): Carthesian2DCoordinate =
            { X = l.X + r.X; Y = l.Y + r.Y }
        

    type Carthesian3DCoordinate with
        static member Origin = {X = 0.0; Y = 0.0; Z = 0.0 }
        static member (+) (l: Carthesian3DCoordinate, r: Carthesian3DCoordinate) =
            { X = l.X + r.X; Y = l.Y + r.Y; Z = l.Z + r.Z }
        

    type SphericalCoordinate with
        static member Origin = { Radius = 0.0; Theta = 0.0<Radian>; Phi = 0.0<Radian> }
        static member (+) (l: SphericalCoordinate, r: SphericalCoordinate) =
            let c1 = ConvertToCarthesian3D l
            let c2 = ConvertToCarthesian3D r
            ConvertToSpherical (c1 + c2)
        

    type PolarCoordinate with
        static member Origin = {Radius = 0.0; Theta = 0.0<Radian> }
        static member (+)(l:PolarCoordinate, r:PolarCoordinate) =
            let c1 = ConvertToCarthesian2D l
            let c2 = ConvertToCarthesian2D r
            ConvertToPolar (c1 + c2)
        

    let addWhiteNoise (d: IContinuousDistribution) c = 
            match c with
            | Carthesian2D c2D  -> c2D +  { X = d.Sample(); Y = d.Sample(); } |> Carthesian2D
            | Carthesian3D c3D  -> c3D + { X = d.Sample(); Y = d.Sample(); Z = d.Sample()} |> Carthesian3D
            | Spherical cSph    -> cSph + { Radius = d.Sample(); Theta = d.Sample() * 1.0<Radian>; Phi = d.Sample() * 1.0<Radian> } |> Spherical
            | Polar cPl         -> cPl + { Radius = d.Sample(); Theta = d.Sample() * 1.0<Radian>; } |> Polar

    [<AbstractClass>]
    type Sensor (position: Coordinate, scanFrequency:float<Hz>) =
        member this.Position = position
        member this.ScanFrequency = scanFrequency

        abstract member Scan': float -> Option<Coordinate array>

        member this.Scan t =
            let d = new Normal()
            if (t % (this.ScanFrequency * 1.0<1/Hz>) <> 0.0) then 
                None
            else
                match this.Scan' t with
                | None -> None
                | Some ms -> ms |> Array.map (addWhiteNoise d) |> Some

    type MockSensor(ts: Coordinate[][], position: Coordinate, scanFrequency:float<Hz>) =
        inherit Sensor(position, scanFrequency)
        member this.Trajectories = ts

        override this.Scan' t =
            None               