import { SlideComponent } from "../components/slide/slide.component";

export interface SlideItem {
  slide: SlideComponent;
  isCompletedNext: () => boolean;
  isCompletedPrev: () => boolean;
  next?: () => any;
  prev?: () => any;
  start?: () => any;
  end?: () => any;
}
