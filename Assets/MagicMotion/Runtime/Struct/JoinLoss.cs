namespace MagicMotion
{
    internal struct JoinLoss
    {
        public float positionloss;
        public float muscleloss;
        public float lookAtloss;
        public float colliderloss;
        public float positionChangeloss;
        public float muscleChangeloss;
        internal float lossSum;

        public void Clacloss()
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
        }

        public override string ToString()
        {
            return lossSum.ToString();
        }
    }
}