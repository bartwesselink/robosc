import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ConclusionVideoComponent } from './conclusion-video.component';

describe('ConclusionFinalComponent', () => {
  let component: ConclusionVideoComponent;
  let fixture: ComponentFixture<ConclusionVideoComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ ConclusionVideoComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(ConclusionVideoComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
