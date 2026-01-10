declare module "react-plotly.js" {
  import * as React from "react";

  export interface PlotParams {
    data?: any;
    layout?: any;
    frames?: any;
    config?: any;
    style?: React.CSSProperties;
    className?: string;
    useResizeHandler?: boolean;
    debug?: boolean;
    onInitialized?: (figure: any, graphDiv: any) => void;
    onUpdate?: (figure: any, graphDiv: any) => void;
    onPurge?: (figure: any, graphDiv: any) => void;
    onError?: (err: any) => void;
    onClick?: (event: any) => void;
    onHover?: (event: any) => void;
    onUnhover?: (event: any) => void;
    onRelayout?: (event: any) => void;
    onRestyle?: (event: any) => void;
    onSelected?: (event: any) => void;
    onDeselect?: (event: any) => void;
    onDoubleClick?: (event: any) => void;
    onAfterPlot?: (event: any) => void;
    onAnimatingFrame?: (event: any) => void;
    onAnimated?: (event: any) => void;
    onRedraw?: (event: any) => void;
    onAutoSize?: (event: any) => void;
    onSunburstClick?: (event: any) => void;
    onLegendClick?: (event: any) => void;
    onLegendDoubleClick?: (event: any) => void;
    onSliderChange?: (event: any) => void;
    onSliderEnd?: (event: any) => void;
    onSliderStart?: (event: any) => void;
    onTransitioning?: (event: any) => void;
    onTransitionInterrupted?: (event: any) => void;
    onWebGlContextLost?: (event: any) => void;
  }

  const Plot: React.ComponentType<PlotParams>;
  export default Plot;
}