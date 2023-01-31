import { ComponentFixture, TestBed } from '@angular/core/testing';

import { SectionOverviewItemComponent } from './section-overview-item.component';

describe('SectionOverviewItemComponent', () => {
  let component: SectionOverviewItemComponent;
  let fixture: ComponentFixture<SectionOverviewItemComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ SectionOverviewItemComponent ]
    })
    .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(SectionOverviewItemComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
