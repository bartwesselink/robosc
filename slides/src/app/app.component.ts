import { AfterContentInit, AfterViewInit, Component, ContentChildren, HostListener, OnInit, QueryList, ViewChildren } from '@angular/core';
import { SectionComponent } from './components/section/section.component';
import { VisibleComponent } from './models/visible-component';
import { first, startWith } from 'rxjs/operators';
import { ActivatedRoute, Router } from '@angular/router';
import { autohideCursor } from '@defaude/autohide-cursor';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.scss']
})
export class AppComponent implements OnInit, AfterViewInit {
  @ViewChildren(SectionComponent) public sections: QueryList<SectionComponent>;
  public scale = 0;
  public x = 1;
  public y = 1;
  private active: VisibleComponent;
  public section = false;
  public activeSection: SectionComponent;
  public started = false;
  public ended = false;
  private skipParams = false;

  constructor(private router: Router, private activatedRoute: ActivatedRoute) {
    this.updateScale();
  }

  ngOnInit() {
    this.activatedRoute
      .queryParams
      .subscribe(params => {
        if (this.skipParams) return;

        this.x =  !!params.x ? +params.x : 1;
        this.y =  !!params.y ? +params.y : 1;
        this.started = params.started === 'true';
        this.ended = params.ended === 'true';
      });

    this.updateScale();

    autohideCursor(5000);
  }

  ngAfterViewInit() {
    this.sections.changes.pipe(startWith(undefined)).subscribe(() => this.updateSectionOffsets());
    setTimeout(() => this.updateActiveSlide());
  }

  @HostListener('window:resize')
  updateScale() {
    const viewportWidth = Math.max(document.documentElement.clientWidth || 0, window.innerWidth || 0);
    const viewportHeight = Math.max(document.documentElement.clientHeight || 0, window.innerHeight || 0);

    const [sourceWidth, sourceHeight] = [1920, 1080];

    const factor = Math.min(
      viewportWidth / sourceWidth,
      viewportHeight / sourceHeight
    );

    this.scale = factor;
  }

  @HostListener('window:keyup', ['$event.key'])
  handleKeyboardNavigation(key: string) {
    switch (key) {
      case 'ArrowLeft':
      case 'PageUp':
        this.previous();
        break;
      case 'ArrowRight':
      case 'PageDown':
        this.next();
        break;
    }
  }

  get currentPage() {
    return this.x + this.y - 1;
  }

  private updateSectionValues() {
    if (this.y > 1) {
      const current = this.sections.get(this.y - 2);
      const next = this.sections.get(this.y - 1);

      let sumX = 0;

      for (const item of this.sections) {
        if (item === current) {
          break;
        }

        sumX += item.slides.length;
      }

      let relative = this.x - sumX;

      if (relative > current.slides.length) {
        this.activeSection = next;
        this.section = true;
      } else {
        this.activeSection = current;
        this.section = false;
      }
    } else {
      this.activeSection = this.sections.get(0);
      this.section = true;
    }
  }

  private updateActiveSlide() {
    this.updateSectionValues();
    const section = this.activeSection;

    const currentActive = this.active;

    if (currentActive) {
      clearTimeout(currentActive.reset);
      currentActive.reset = setTimeout(() => {
        currentActive.active = false;
      }, 500);
    }

    if (this.section) {
      clearTimeout(section.reset);
      this.active = section;
      section.active = true;
    } else {
      const activeSlide = this.currentRelativeX;

      const slide = section.slides.get(activeSlide - 1).slide;

      clearTimeout(slide.reset);
      this.active = slide;
      slide.active = true;
    }
  }

  private updateSectionOffsets() {
    let currentX = 0, currentY = 0;

    for (const item of this.sections) {
      const x = currentX;
      const y = currentY;

      setTimeout(() => {
        item.offsetX = x;
        item.offsetY = y;
      });

      currentX += item.slides.length;
      currentY -= 1;
    }

    setTimeout(() => {
      this.updateSectionValues();
    })
  }

  private next() {
    if (!this.started) {
      this.started = true;
      this.updateQueryParameters();
      return;
    }

    if (this.section) {
      this.y += 1;
    } else {
      const activeSlide = this.currentRelativeX;
      const slide = this.activeSection.slides.get(activeSlide - 1);

      if (!slide.isCompletedNext()) {
        slide.next();
        return;
      }

      if (this.currentRelativeX < this.activeSection.slides.length || this.activeSection !== this.sections.last) {
        this.x += 1;
      } else {
        this.ended = true;
        this.updateQueryParameters();
        return;
      }
    }

    this.updateActiveSlide();
    this.updateQueryParameters();

    // Check if we should reset
    if (!this.section) {
      const activeSlide = this.currentRelativeX;
      const slide = this.activeSection.slides.get(activeSlide - 1);

      slide.start();
    }
  }

  private previous() {
    if (this.ended) {
      this.ended = false;
      this.updateQueryParameters();
      return;
    }

    if (!this.section) {
      const activeSlide = this.currentRelativeX;
      const slide = this.activeSection.slides.get(activeSlide - 1);

      if (!slide.isCompletedPrev()) {
        slide.prev();
        return;
      }
    }

    if (this.currentRelativeX === 1) {
      this.y -= 1;
    } else {
      if (this.y > 1) {
        this.x -= 1;
      } else {
        this.started = false;
        this.updateQueryParameters();
        return;
      }
    }

    this.updateActiveSlide();
    this.updateQueryParameters();

    // Check if we should reset
    if (!this.section) {
      const activeSlide = this.currentRelativeX;
      const slide = this.activeSection.slides.get(activeSlide - 1);

      slide.end();
    }
  }

  get currentRelativeX() {
    if (this.section) {
      return 0;
    }

    let sumX = 0;
    let section = this.activeSection;

    for (const item of this.sections) {
      if (section === item) {
        break;
      }

      sumX += item.slides.length;
    }

    return this.x - sumX;
  }

  updateQueryParameters() {
    this.skipParams = true;
    this.router.navigate(
      [],
      {
        relativeTo: this.activatedRoute,
        queryParams: {
          x: this.x,
          y: this.y,
          started: this.started ? 'true' : 'false',
          ended: this.ended ? 'true' : 'false',
        },
        queryParamsHandling: 'merge',
      });
  }
}
