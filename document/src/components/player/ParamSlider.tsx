import {useEffect, useState} from "react";

// sandbox 수치 파라미터용 슬라이더. 드래그 중에는 값 표시만 갱신하고, 놓는 순간
// (pointerup/keyup) 커밋해 엔진을 재실행한다 — 라이브 planner 재계산이 드래그를
// 버벅이게 하지 않도록.
interface ParamSliderProps {
    label: string;
    value: number;
    min: number;
    max: number;
    step: number;
    onCommit: (v: number) => void;
    // 값 표시 형식 (기본: step<1 이면 소수 표시)
    format?: (v: number) => string;
    width?: number;
}

const ParamSlider = ({label, value, min, max, step, onCommit, format, width = 150}: ParamSliderProps) => {
    const [live, setLive] = useState(value)
    useEffect(() => setLive(value), [value])
    const fmt = format ?? ((v: number) => step < 1 ? v.toFixed(2).replace(/\.?0+$/, "") : String(v))
    const commit = () => {
        if (live !== value) onCommit(live)
    }
    return (
        <label className="flex items-center gap-1.5 text-xs text-muted whitespace-nowrap">
            <span>{label}</span>
            <input type="range" min={min} max={max} step={step} value={live}
                   onChange={(e) => setLive(parseFloat(e.target.value))}
                   onPointerUp={commit}
                   onKeyUp={commit}
                   onBlur={commit}
                   className="accent-[var(--accent)]" style={{width}}
                   aria-label={label}/>
            <span className="tabular-nums font-semibold min-w-[2.5rem]">{fmt(live)}</span>
        </label>
    )
}

export default ParamSlider
