import { SlideComponent } from "../components/slide/slide.component";
import { SlideItem } from "./slide-item";

export abstract class AbstractSlide implements SlideItem {
  abstract slide: SlideComponent;
  public step = 0;
  abstract max: number;

  isCompletedNext() {
    return this.step >= this.max;
  }

  isCompletedPrev() {
    return this.step <= 0;
  }

  prev() {
    this.step -= 1;
  }

  next() {
    this.step += 1;
  }

  start() {
    this.step = 0;
  }

  end() {
    this.step = this.max;
  }
}
