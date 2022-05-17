namespace MagicMotion
{
    internal struct JointLoss
    {
        public double positionloss;
        public double muscleloss;
        public double lookAtloss;
        public double colliderloss;
        public double positionChangeloss;
        public double muscleChangeloss;
        internal double lossSum;

/*        public void Clacloss()
        {
            lossSum = positionloss + muscleloss + lookAtloss + colliderloss + positionChangeloss + muscleChangeloss;
            Clear();
        }

        public void Clear()
        {
            positionloss = 0;
            muscleloss = 0;
            lookAtloss = 0;
            colliderloss = 0;
            positionChangeloss = 0;
            muscleChangeloss = 0;
        }*/

        public override string ToString()
        {
            return lossSum.ToString();
        }
    }
}