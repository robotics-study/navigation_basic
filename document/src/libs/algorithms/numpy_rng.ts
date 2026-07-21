// numpy `default_rng(seed)`의 정확한 미러 (SeedSequence 엔트로피 믹싱 + PCG64
// XSL-RR + Generator.random/uniform). 저장소의 sampling planner 들이 numpy RNG로
// 표본을 뽑으므로, 같은 seed에서 같은 난수열을 재현해야 라이브 엔진이 python
// demo와 표본 단위까지 일치한다 (parity 하니스의 [exact] 전제).
const MASK32 = 0xffffffffn;
const MASK64 = 0xffffffffffffffffn;
const MASK128 = (1n << 128n) - 1n;

// SeedSequence 상수 (O'Neill의 randutils seed_seq_fe 계열).
const INIT_A = 0x43b0d7e5n;
const MULT_A = 0x931e8875n;
const INIT_B = 0x8b51f9ddn;
const MULT_B = 0x58f38dedn;
const MIX_MULT_L = 0xca01f9ddn;
const MIX_MULT_R = 0x4973f715n;
const XSHIFT = 16n;
const POOL_SIZE = 4;

// seed(비음수 정수)를 32-bit little-endian 워드 열로 분해한다.
const entropyWords = (seed: number): bigint[] => {
    let v = BigInt(seed)
    if (v === 0n) return [0n]
    const out: bigint[] = []
    while (v > 0n) {
        out.push(v & MASK32)
        v >>= 32n
    }
    return out
}

// SeedSequence(seed).generate_state(nWords, uint64) — 64-bit 워드 nWords개.
export function seedSequenceState(seed: number, nWords: number): bigint[] {
    const entropy = entropyWords(seed)
    const pool: bigint[] = new Array(POOL_SIZE).fill(0n)

    let hashConst = INIT_A
    const hash = (value: bigint): bigint => {
        value = (value ^ hashConst) & MASK32
        hashConst = (hashConst * MULT_A) & MASK32
        value = (value * hashConst) & MASK32
        value = (value ^ (value >> XSHIFT)) & MASK32
        return value
    }
    const mix = (x: bigint, y: bigint): bigint => {
        let result = (((MIX_MULT_L * x) & MASK32) - ((MIX_MULT_R * y) & MASK32)) & MASK32
        result = (result ^ (result >> XSHIFT)) & MASK32
        return result
    }

    for (let i = 0; i < POOL_SIZE; i++) {
        pool[i] = hash(i < entropy.length ? entropy[i] : 0n)
    }
    for (let iSrc = 0; iSrc < POOL_SIZE; iSrc++) {
        for (let iDst = 0; iDst < POOL_SIZE; iDst++) {
            if (iSrc !== iDst) pool[iDst] = mix(pool[iDst], hash(pool[iSrc]))
        }
    }
    for (let iSrc = POOL_SIZE; iSrc < entropy.length; iSrc++) {
        for (let iDst = 0; iDst < POOL_SIZE; iDst++) {
            pool[iDst] = mix(pool[iDst], hash(entropy[iSrc]))
        }
    }

    // generate_state: 32-bit 워드 2n개를 뽑아 little-endian 쌍으로 64-bit 조립.
    const words32: bigint[] = []
    let genConst = INIT_B
    let iSrc = 0
    for (let k = 0; k < nWords * 2; k++) {
        let dataVal = pool[iSrc % POOL_SIZE]
        iSrc++
        dataVal = (dataVal ^ genConst) & MASK32
        genConst = (genConst * MULT_B) & MASK32
        dataVal = (dataVal * genConst) & MASK32
        dataVal = (dataVal ^ (dataVal >> XSHIFT)) & MASK32
        words32.push(dataVal)
    }
    const out: bigint[] = []
    for (let k = 0; k < nWords; k++) {
        out.push(words32[2 * k] | (words32[2 * k + 1] << 32n))
    }
    return out
}

// PCG64 (XSL-RR 128/64) — numpy default_rng의 BitGenerator.
const PCG_MULT = 0x2360ed051fc65da44385df649fccf645n;

export class NumpyRandom {
    private state: bigint;
    private readonly inc: bigint;

    constructor(seed: number) {
        const w = seedSequenceState(seed, 4)
        const initState = (w[0] << 64n) | w[1]
        const initSeq = (w[2] << 64n) | w[3]
        this.inc = ((initSeq << 1n) | 1n) & MASK128
        this.state = 0n
        this.step()
        this.state = (this.state + initState) & MASK128
        this.step()
    }

    private step(): void {
        this.state = (this.state * PCG_MULT + this.inc) & MASK128
    }

    nextUint64(): bigint {
        this.step()
        const hi = this.state >> 64n
        const lo = this.state & MASK64
        const xored = (hi ^ lo) & MASK64
        const rot = this.state >> 122n
        return ((xored >> rot) | ((xored << (64n - rot)) & MASK64)) & MASK64
    }

    // numpy next_double: 상위 53비트 / 2^53.
    random(): number {
        return Number(this.nextUint64() >> 11n) / 9007199254740992
    }

    uniform(low: number, high: number): number {
        return low + (high - low) * this.random()
    }
}
